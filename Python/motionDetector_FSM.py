## 
# @file motionDetector_FSM.py
# @author Leonardo Brandão Borges de Freitas (contato.leonardobbf@gmail.com)
# @date 26 Mar 2021
# @brief Classe Python para detecção de movimento baseado na máquina de estados do meu TCC
#   
#  Diagrama de bloco da FSM: 
#

from re import L
from cv2 import (VideoCapture, VideoWriter_fourcc, imshow, waitKey, 
                 blur, INTER_AREA, COLOR_BGR2GRAY, cvtColor, absdiff, 
                 GaussianBlur, threshold, destroyAllWindows) 
import numpy as np
from exception import CaptureError, Motion, NoMotion
from queue import Queue, Full, Empty
from threading import Thread, Event
from time import sleep

class FrameCapture():

    ## @brief Instanciador da classe FrameCapture
    #   
    #
    def __init__(self, 
                 capture_path,
                 fifo_out,
                 fps,
                 fps_percent,
                 resolution,
                 event_time):

        # Define os parametros de captura
        self.path = capture_path
        self.source_fps = fps 
        self.fps_percent = fps_percent
        self.source_resolution = resolution
        self.event_time = event_time #segundos

        # Apresenta os parâmetros na tela
        print(f"FrameCapture:\n\
        \tpath = {self.path},\n\
        \tsource_fps = {self.source_fps},\n\
        \tresolution = {self.source_resolution},\n\
        \treal_fps = {self.source_fps*fps_percent/100}\n\
        \tevent_time = {self.event_time}")

        # Tratando o caminho de captura pra webcam
        if self.path == '0':
            self.path = 0

        # Instancia um objeto de captura de vídeo (via OpenCV)
        self.cap = VideoCapture(self.path)

        # Coloca como self a fila de saída
        self.fifo = fifo_out

        # Inicializa a flag de continuidade da classe
        self.should_continue = False

        # Instancia a thread de captura
        self.capture_thread = Thread(target = self.run, 
                                     name = 'captureThread', 
                                     daemon=True)

        # Instancia um controlador de eventos para thread de captura
        self.ready = Event()

    ## @brief Método para inicializar as operações da classe
    #
    def start(self):

        # Verifica se a thread está pausada
        if self.should_continue and not self.ready.isSet():

            # Se estiver, libera o evento da thread
            self.ready.set()

            print("start: retornando as operações da FC")

        # Se não estiver pausado, nem iniciado:
        elif not self.should_continue:

            # Levanta a flag de continuidade
            self.should_continue = True

            # Inicia a thread de captura
            self.capture_thread.start()

            # Libera os eventos da thread
            self.ready.set()

            print("start: Iniciando as operações da FC")

    ## Método de parada das operações da classe
    #
    def stop(self):
        
        # Se a classe foi iniciada
        if self.should_continue:

            # Abaixa a flag de continuidade
            self.should_continue = False

            # e libera o evento da thread evitando travamentos na finalização
            self.ready.set()

            print("stop: parando as operações da FC")

    ## @brief Método de liberação de memória da classe
    #
    def free(self):

        # Se a classe estiver parada
        if not self.should_continue:
            # Libera o objeto VideoCapture
            self.cap.release()

            print("free: Liberando o objeto VideoCapture ")

    ## @brief Rotina para a thread de captura e enfileiramento de frames
    #
    def run(self):

        print("Iniciando a thread de captura e enfileiramento ...")

        # Laço principal de captura
        while True:

            # Aguarda a autorização
            self.ready.wait()
            #print('run: ready.wait')

            # Verifica a flag de continuidade
            if not self.should_continue:
                print('A thread de captura e enfileiramento não deve continuar !')
                # Se for falsa, o laço é quebrado
                break

            try:

                # Captura o frame
                frame = self.capture() 
                #print("run: capture")

                # Coloca o frame capturado na fila 
                self.fifo.put(frame)
                #print('run: fifo.put(frame)')

            # Se um erro de captura levantado
            except CaptureError as err:
                # Informa o ocorrido
                print(f'run: {err}')
                # e pausa os eventos da thread
                self.ready.clear()

            # Se a fila estiver cheia
            except Full as err:
                # Informa o corrido 
                print(f'run: {err}, qsize = {self.fifo.qsize()}')
                
                self.ready.clear()

    ## @brief Método para capturar um frame
    #
    def capture(self):

        # Para o caso do caminho de captura apontar para um vídeo mp4
        if self.path.find(".mp4") != -1:

            # Agarra um frame do fluxo
            ret = self.cap.grab()

            # Se o frame não foi agarrado com sucesso
            if not ret:
                raise CaptureError('grab error (mp4)')

            # Captura e armazena o frame em memória 
            ret, frame = self.cap.retrieve()

            # Se o frame não foi capturado com sucesso 
            if not ret:
                raise CaptureError('retrieve error (mp4)')

        # Para o caso de apontar para um streamer (Câmera ou webcam)
        else:

            # Agarra os frames espaçados de acordo com o fps_percent
            for j in range(int(100/self.fps_percent)):
                ret = self.cap.grab()

                if not ret:
                    raise CaptureError('grab error')
            
            # Captura e armazena o frame em memória 
            ret, frame = self.cap.retrieve()
            
            if not ret:
                raise CaptureError('retrieve error')

        # Apresenta o frame capturado na tela
        imshow('frame', frame)
        waitKey(100)

        # retorna o frame capturado
        return frame

    
class MotionDetector():

    ## @brief Instanciador da classe MotionDetector
    #   
    #
    def __init__(self,
                chunk_lines = 30,
                chunk_columns = 40,
                threshold = 15):

        # Define os parametros de detecção
        self.chunk_lines = chunk_lines
        self.chunk_columns = chunk_columns
        self.thresh = threshold
        
        print(f"MotionDetector:\n\
        \tthreshold = {self.thresh},\n\
        \tchunk_lines = {self.chunk_lines},\n\
        \tchunk_columns = {self.chunk_columns}")

        ''' CONTADORES '''
        # Contador total de pixels
        self.total_bytes = 0

        # Contador de bytes
        self.b = 0
        print(f'Contador de bytes (b) = {self.b}')

        # Seletora do mux de registradores
        self.sel = 0 
        print(f'Seletora (sel) = {self.sel}')

        # Contador de linhas de pixel 
        self.lp = 0
        print(f'Contador de linhas de pixel (lp) = {self.lp}')

        # Contador de linhas de chunks
        self.lc = 0
        print(f'Contador de linhas de chunks (lc) = {self.lc}')

        # Contador de linhas do segundo frame
        self.l = 0
        print(f'Contador de linhas do segundo frame (l) = {self.l}')

        # Acumulador de movimento para os 1200 chunks
        self.am = 0
        print(f'Acumulador de Movimento (am) = {self.am}')


        ''' REGISTRADORES '''
        # Registradores de entrada
        self.reg = np.zeros(self.chunk_columns, dtype=np.uint16)
        print(f'Registradores de entrada = reg{self.reg.shape}')

        # Matriz de chunks 
        self.mc = np.zeros((self.chunk_lines, self.chunk_columns), dtype=int)
        print(f'Matriz de Chunks = MC{self.mc.shape}')

        # Vetor de Movimento
        self.vm = np.zeros(self.chunk_columns, dtype=int)
        print(f'Vetor de Movimento = VM{self.vm.shape}')
     
    def pegaPixel(self, I):

        # Aloca o valor do pixel na variável vp
        vp = I[self.b]

        # retorna valor do pixel
        return vp

    def acumulador(self, vp):

        # Aculuma o valor de intensidade do pixel lido
        self.reg[self.sel] += vp

        # Itera o contador de pixels
        self.b += 1

        self.total_bytes +=1

    def iteraSel(self):

        # Itera a seletora
        self.sel += 1

    def zeraSel(self):

        # Zera a seletora
        self.sel = 0

        # Itera o contador de linhas de pixel
        self.lp += 1

    def shiftRight(self):

        # Zera o contador de linhas de pixel
        self.lp = 0

        # Executa o deslocamento a direita em todos os registradore
        for i in range(len(self.reg)):
            self.reg[i] = self.reg[i] >> 8  

    def alocaMedias(self):

        # para cada posição do registrador
        for i in range(len(self.reg)):
            # aloca a média na matriz de chunks
            self.mc[self.lc][i] = self.reg[i]
            # zera o registrador
            self.reg[i] = 0

        # Itera o contador de linhas de chunks
        self.lc += 1

    def mediasDiff(self):

        # para cada posição do registrador
        for i in range(len(self.reg)):

            # Subtrai a média de um quadrante do primeiro frame 
            # com a média de um quadrante do segundo frame
            self.mc[self.l][i] = self.mc[self.l][i] - self.reg[i]

            # A posição do registrador é zerada
            self.reg[i] = 0

    def complementoDe2(self):

        # para cada posição do registrador
        for i in range(len(self.reg)):

            # Se a diferença entre as médias for negativa
            if self.mc[self.l][i] < np.uint8(0):
                # Aplica o complemento de dois para encontrar o módulo
                self.mc[self.l][i] = self.mc[self.l][i] * (-1) 

    def limiarizacao(self):

        # para cada posição do registrador
        for i in range(len(self.reg)):

            # Se o modulo da diferença entre os chunks do primeiro e do segundo frame
            # for maior que o limiar instânciado 
            if self.mc[self.l][i] > self.thresh:
                # A posição atual do veor de movimento recebe 1
                self.vm[i] = 1
            # Caso contrário,
            else:
                # Recebe 0
                self.vm[i] = 0

        # O contador de colunas da matriz de chunks é iterado
        self.l += 1

    def contaUm(self):

        # Acumula em am os 1s do vetor de movimento
        for value in self.vm:
            self.am+=value


    def verificaMovimento(self):

        #print(f'am = {self.am}')

        # Se pelo menos 25% dos chunks (1200/4 = 300) acusarem movimento
        if self.am >= 300:
            # Retorna que houve movimento
            return 1

        # Se não,
        else:
            # Retorna que não houve movimento
            return 0

    def reset(self):

        # Zera os contadores 
        self.b = 0
        self.sel = 0 
        self.lp = 0
        self.lc = 0
        self.l = 0
        self.am = 0

        # Zera os registradores
        self.reg = np.zeros(self.chunk_columns, dtype=np.uint16)
        self.mc = np.zeros((self.chunk_lines, self.chunk_columns), dtype=int)
        self.vm = np.zeros(self.chunk_columns, dtype=np.uint16)

## @brief Função para concatenar dois frames em escala de cinza
#
def concatenateGrayPair(frame1, frame2):

        # Coloca os frames em escala de cinza
        frame1 = cvtColor(frame1, COLOR_BGR2GRAY)
        frame2 = cvtColor(frame2, COLOR_BGR2GRAY)

        # Aplica um embaçamento nos frames
        #frame1 = blur(frame1, (5, 5))
        #frame2 = blur(frame2, (5, 5))

        frame1 = np.concatenate(frame1)
        frame2 = np.concatenate(frame2)

        # Concatena os frames em um vetor unidimensional 
        I = np.append(frame1, frame2)

        # retonar o vetor de intensidades do frame
        return I

if __name__ == "__main__":

    # Recebendo os argumentos
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("-pth", "--capture_path", required=True, 
    help="Caminho para a fonte de captura")
    ap.add_argument("-fps", "--source_fps", required=False, type=int, default=15,
    help="Velocidade de captura configurado na fonte, em frames por segundo")
    ap.add_argument("-fps_percent", "--fps_percent", required=False, type=int, default=40,
    help="Porcentagem do FPS da fonte utilizada pelo FrameProducer (valor entre 0 e 100)")
    ap.add_argument("-rsl", "--source_resolution", required=False, type=int, nargs="+", default=[640, 480],
    help="Resolução dos frames configurada na fonte (Largura Altura) ")
    ap.add_argument("-event_length", "--event_length", required=False, type=int, default=3,
    help="Tempo de captura dos eventos, em segundo")
    args = vars(ap.parse_args())

    # Instancia uma FIFO para enfileirar os frames capturados
    # Entrada: fc
    # Saída: md 
    fifo = Queue(maxsize=20)

    # Instancia um objeto FrameCapture
    fc = FrameCapture(capture_path = args["capture_path"], 
                      fifo_out = fifo,
                      fps = args["source_fps"], 
                      fps_percent = args["fps_percent"],
                      resolution = args["source_resolution"],
                      event_time = args["event_length"])

    # Instancia um objeto MotionDetector
    md = MotionDetector()

    # Inicia as operações do objeto FrameCapture
    fc.start()

    # Inicializando as variáveis de estado
    #   0: Prepara Frames
    #   1: Pega Pixel
    #   2: Acumulador
    #   3: Itera Seletora
    #   4: Zera Seletora
    #   5: Shift Right
    #   6: Aloca Médias
    #   7: Médias Diff
    #   8: Complemento de 2
    #   9: Limiarização
    #  10: ContaUm
    #  11: Verifica Movimento
    #  12: Reset
    estado_atual = 0
    proximo_estado = 0

    # Entrando em laço infinito
    inloop = True
    while inloop:

        try:

            #   0: Prepara Frames
            if estado_atual == 0:
            
                # Pega um par de frames na fifo
                frame1 = fifo.get()
                frame2 = fifo.get()

                # Prepara o par de frame em escala de cinza e faz a concatenação unidimensional dos pixeis
                # no vetor de intensidades I que é equivalente ao arquivo de pixel utilizado pelo Test Bench em VHDL)
                I = concatenateGrayPair(frame1, frame2)

                # Proximo estado será o Pega Pixel
                proximo_estado = 1
            
            #   1: Pega Pixel
            elif estado_atual == 1:

                # Pega o pixel do vetor concatenado
                vp = md.pegaPixel(I)

                # Proximo estado será o Acumulador
                proximo_estado = 2

            #   2: Acumulador
            elif estado_atual == 2:

                #print(f'acumulador: b = {md.b}')

                # Acumula o pixel no registrador
                md.acumulador(vp)

                # Se ainda não completou a linha do chunk (16 bytes)
                if md.b % 16 != 0:
                    # Proximo estado será o Pega Pixel
                    proximo_estado = 1

                # mas se já completou a linha do chunk
                elif md.b % 16 == 0:
                    # Proximo estado será o Itera Seletora
                    proximo_estado = 3

            #   3: Itera Seletora
            elif estado_atual == 3:

                #print(f'Itera seletora: sel = {md.sel}')

                # Itera a seletora para mudar de registrador
                md.iteraSel()

                # Enquanto a seletora for menor que 40
                if md.sel < 40:
                    # Proximo estado será o Pega Pixel
                    proximo_estado = 1

                elif md.sel == 40:
                    # Proximo estado será o Zera Seletora
                    proximo_estado = 4

            #   4: Zera Seletora
            elif estado_atual == 4:

                #print(f'Zera Seletora: l = {md.l}')

                # Zera a seletora e itera o contador de linhas de pixel
                md.zeraSel()

                # Enquanto não tiver preenchido as 16 linhas de pixel para os 40 registradores 
                if md.lp < 16:
                    # Proximo estado será o Pega Pixel
                    proximo_estado = 1

                # Mas se já tiver preenchido as 16 linhas de pixel para os 40 registradores
                elif md.lp == 16:
                    # Proximo estado será o shift right
                    proximo_estado = 5

            #   5: Shift Right
            elif estado_atual == 5:

                # Executa os deslocamentos para a direita em todos os registradores
                # calculando a média dos pixeis acumulados
                md.shiftRight()

                # Enquanto não tiver alocado as 30 linhas de chunks do primeiro frame 
                if md.lc < 30:
                    #print(f'shift right: b = {md.total_bytes}, lc = {md.lc}')
                    # Proximo estado será o Aloca Médias
                    proximo_estado = 6
                
                # Mas se ja tiver alocado todas as linhas de chunks do primeiro frame
                if md.lc == 30: 
                    # Proximo estado será o Médias Diff
                    proximo_estado = 7

            #   6: Aloca Médias
            elif estado_atual == 6:

                # Aloca as médias dos 40 chunks na matriz de chunks
                md.alocaMedias()

                #print(md.mc) 

                # Proximo estado será o Pega Pixel
                proximo_estado = 1
 
            #   7: Médias Diff
            elif estado_atual == 7:
                
                # Executa a subtração entre as médias dos chunks 
                # do segundo frame com o primeiro.
                md.mediasDiff()

                # o próximo estado será o complemento de 2
                proximo_estado = 8

            #   8: Complemento de dois
            elif estado_atual == 8:

                # Aplica o complemento de dois nos resultados negativos 
                md.complementoDe2()

                # Proximo estado será o Limiarização
                proximo_estado = 9

            #   9: Limiarização
            elif estado_atual == 9:

                # Aplica a limiarização aos módulos calculados
                md.limiarizacao()

                # Proximo estado será o Conta Um
                proximo_estado = 10
    
            #   10: Conta Um
            elif estado_atual == 10:

                # Conta a quantidade de uns no vetor de movimento
                md.contaUm()

                print(f'Pixels Lidos = {md.total_bytes}')
                print(f'VM[40] = {md.vm}')
                print(f'am = {md.am}')

                # Enquanto não tiver completado as 30 linhas de chunks do segundo frame 
                if md.l < 30:
                    # o proximo estado será o Pega Pixel
                    proximo_estado = 1

                # Se tiver completado as 30 linhas de chunks do segundo frame
                elif md.l == 30:
                    # Próximo estado será o Verifica Movimento
                    proximo_estado = 11
        
            # 11: Verifica Movimento
            elif estado_atual == 11:

                # verifica se há movimento, comparando a matriz de chunks atual com a passada
                movimento = md.verificaMovimento()

                print(f'am = {md.am}')
                print(f'result = {movimento}')

                # Proximo estado será o Reset
                proximo_estado = 12
            
            #  12: Reset
            elif estado_atual == 12:

                # Reseta as variáveis de controle
                md.reset()

                # Proximo estado será o Prepara frame
                proximo_estado = 0   

            # Atualiza o estado atual
            estado_atual = proximo_estado

        # Se a fila estivar vazia
        except Empty as err:
            # Informa o ocorrido
            print(f'main: {err}')

        # Se receber a interrupção de sistema Ctrl+C
        except KeyboardInterrupt: #SIGINT
            # Coloca a variável de laço em falso
            inloop = False
            # Encerra o objeto
            fc.stop()
            fc.free()
        
    destroyAllWindows()
        






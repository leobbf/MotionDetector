## 
# @file motionDetector_FSM.py
# @author Leonardo Brandão Borges de Freitas (contato.leonardobbf@gmail.com)
# @date 26 Mar 2021
# @brief Classe Python para detecção de movimento baseado na máquina de estados do meu TCC1
#   
#  Diagrama de bloco da FSM: https://drive.google.com/file/d/18bM7gxYAv3rrYRTcxtIaTuT-0rMBh7-F/view?usp=sharing
#

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
                 fifo_out):

        # Define os parametros de captura
        self.source_fps = 15 
        self.fps_percent = 20
        self.source_resolution = [640, 480]
        self.event_time = 5 #segundos

        # Apresenta os parâmetros na tela
        print(f"FrameCapture:\n\
        \tpath = {capture_path},\n\
        \tfps = {self.source_fps},\n\
        \tresolution = {self.source_resolution}")

        # Tratando o caminho de captura pra webcam
        if capture_path == '0':
            capture_path = 0

        # Instancia um objeto de captura de vídeo (via OpenCV)
        self.cap = VideoCapture(capture_path)

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
        
        # Se a classe for iniciada
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
                # e quebra o laço
                break

            # Se a fila estiver cheia
            except Full as err:
                # Informa o corrido 
                print(f'run: {err}, qsize = {self.fifo.qsize()}')
                # e pausa os eventos da thread
                self.ready.clear()

    ## @brief Método para capturar um frame
    #
    def capture(self):

        # Captura o frame 
        for j in range(int(100/self.fps_percent)):
            ret = self.cap.grab()

            # Verifica se o frame foi capturado com sucesso
            if not ret:
                # Se não foi, lança a exceção CaptureError
                raise CaptureError('Capture Error')
        
        # Se o frame foi capturado com sucesso
        # Executa o método retrieve para armazenar o frame capturado
        _, frame = self.cap.retrieve()

        imshow('frame', frame)
        waitKey(0)
        
        # retorna o frame capturado
        return frame

class MotionDetector():

    ## @brief Instanciador da classe MotionDetector
    #   
    #
    def __init__(self):

        # Define os parametros de detecção
        self.thresh = 15
        self.chunk_lines = 30
        self.chunk_columns = 40
        
        print(f"MotionDetector:\n\
        \tthreshold = {self.thresh},\n\
        \tchunk_lines = {self.chunk_lines},\n\
        \tchunk_columns = {self.chunk_columns}")

        # Contador de bytes
        self.b = 0
        print(f'Contador de bytes = {self.b}')

        # Contador de linhas completas
        self.l = 0
        print(f'Contador de linhas completas = {self.l}')

        # Seletora do mux de registradores
        self.sel = 0 
        print(f'Seletora = {self.sel}')

        # Contador de linhas da matriz de chunks
        self.j = 0
        print(f'Contador de linhas da matriz de chunks = {self.j}')

        # Contador de colunas da matriz de chunks
        self.c = 0
        print(f'Contador de colunas da matriz de chunks = {self.c}')

        # Registradores para a primeira linha de chunks
        self.reg = np.zeros(self.chunk_columns, dtype=np.uint16)
        print(f'registradores = {self.reg.shape}')

        # Matriz de média dos chunks 
        self.mc = np.zeros((self.chunk_lines, self.chunk_columns), dtype=int)
        print(f'Matriz de Média dos Chunks = {self.mc.shape}')

        # Acumulador de movimento para os 1200 chunks
        self.am = 0
        print(f'acumulador de Movimento = {self.am}')

    def preparaPar(self, frame1, frame2):

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

    def iteraSel(self):

        # Itera a seletora
        self.sel += 1

    def zeraSel(self):

        # Zera a seletora
        self.sel = 0

        # Itera o contador de linhas completas
        self.l += 1

    def shiftRight(self):

        # Executa o deslocamento a direita em todos os registradore
        for i in range(len(self.reg)):
            self.reg[i] = self.reg[i] >> 8

        # Itera o contador de linhas da matriz de chunks
        self.j += 1

        # Zera o contador de linhas completas
        self.l = 0

        # Zera o contador de colunas da matriz de chunks
        self.c = 0

    def alocaMedias(self):

        # para cada posição do registrador
        for i in range(len(self.reg)):
            # aloca a média na matriz de chunks
            self.mc[self.j-1][i] = self.reg[i]
            # zera o registrador
            self.reg[i] = 0

    def mediasDiff(self):

        # Subtrai a média de um quadrante do primeiro frame 
        # com a média de um quadrante do segundo frame
        self.mc[self.j-31][self.c] = self.mc[self.j-31][self.c] - self.reg[self.c]

        # A posição do registrador é zerada
        self.reg[self.c] = 0

    def complementoDe2(self):

        # Se a diferença entre as médias for negativa
        if self.mc[self.j-31][self.c] < np.uint8(0):
            # Aplica o complemento de dois para encontrar o módulo
            self.mc[self.j-31][self.c] = self.mc[self.j-31][self.c] * (-1) 

    def limiarizacao(self):

        # Se o quadrante atual for maior que o quadrante anterior vezes o limiar atribuido
        if self.mc[self.j-31][self.c] > self.thresh:
            # O acumulador de movimento é iterado
            self.am += 1 

        # O contador de colunas da matriz de chunks é iterado
        self.c += 1    

    def verificaMovimento(self):

        print(f'acumulador de movimento = {self.am}')

        # Se pelo menos 25% dos chunks (1200/4 = 300) acusarem movimento
        if self.am >= 300:
            # Retorna que houve movimento
            return 1

        # Se não,
        else:
            # Retorna que não houve movimento
            return 0

    def reset(self):

        # Zera o contador de bytes
        self.b = 0

        # Zera a seletora do mux de registradores
        self.sel = 0 

        # Zera o contador de linhas da matriz de chunks
        self.j = 0

        # Zera o contador de colunas da matriz de chunks
        self.c = 0

        # Zera o acumulador de movimento
        self.am = 0

if __name__ == "__main__":

    # Recebendo os argumentos
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("-pth", "--capture_path", required=True, 
    help="Caminho para a fonte de captura")
    args = vars(ap.parse_args())

    # Instancia uma FIFO para enfileirar os frames capturados
    # Entrada: fc
    # Saída: md 
    fifo = Queue(maxsize=10)

    # Instancia um obejto FrameCapture
    fc = FrameCapture(args["capture_path"], fifo)

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
    #  10: Verifica Movimento
    #  11: Reset
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
                # No vetor de intensidades
                I = md.preparaPar(frame1, frame2)

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

                # Zera a seletora e itera o contador de linhas completas
                md.zeraSel()

                # Enquanto não tiver preenchido as 16 linhas completas para os 40 registradores 
                if md.l < 16:
                    # Proximo estado será o Pega Pixel
                    proximo_estado = 1

                # Mas se já tiver preenchido as 16 linhas completas para os 40 registradores
                elif md.l == 16:
                    # Proximo estado será o shift right
                    proximo_estado = 5

            #   5: Shift Right
            elif estado_atual == 5:

                #print(f'shift right: b = {md.b}, j = {md.j}')

                # Executa os deslocamentos para a direita em todos os registradores
                # calculando a média dos pixeis acumulados
                md.shiftRight()

                # Enquanto não tiver alocado todos os chunks do primeiro frame (307200 bytes)
                if md.b <= 307200: 
                    # Proximo estado será o Aloca Médias
                    proximo_estado = 6
                
                # Mas se ja tiver alocado todos os chunks do primeiro frame
                if md.b > 307200: 
                    # Proximo estado será o Médias Diff
                    proximo_estado = 7

            #   6: Aloca Médias
            elif estado_atual == 6:

                # Aloca as médias dos 40 chunks na matriz de chunks
                md.alocaMedias()

                # Proximo estado será o Pega Pixel
                proximo_estado = 1
 
            #   7: Médias Diff
            elif estado_atual == 7:

                #print(f'Médias Diff: j = {md.j}, c = {md.c}')

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

                # Enquanto o contador de colunas da matriz de chunks for menor que 40 
                if md.c < 40:
                    # o proximo estado será o Médias Diff
                    proximo_estado = 7

                # Se todas as colunas foram comparadas, mas os dois frames ainda não foram lidos por completo
                elif md.c == 40 and md.b < 2*307200:
                    # Próximo estado será o Pega pixel
                    proximo_estado = 1

                # Se todas as colunas foram comparadas e os dois frames foram lidos por completo
                elif md.c == 40 and md.b == 2*307200:
                    # Próximo estado será o verifica movimento
                    proximo_estado = 10

            #  10: Verifica Movimento
            elif estado_atual == 10:

                # verifica se há movimento, comparando a matriz de chunks atual com a passada
                movimento = md.verificaMovimento()

                print(f'Movimento = {movimento}')

                # Proximo estado será o Reset
                proximo_estado = 11
            
            #  11: Reset
            elif estado_atual == 11:

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
        






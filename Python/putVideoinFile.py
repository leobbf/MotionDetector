from cv2 import (VideoCapture, VideoWriter, VideoWriter_fourcc, 
                 cvtColor, COLOR_BGR2GRAY, CAP_FFMPEG,
                 imshow, waitKey, destroyAllWindows)
from threading import Thread, Event
from exception import CaptureError
from queue import Queue, Full, Empty
import numpy as np
from time import sleep


class FrameCapture():

    ## @brief Instanciador da classe FrameCapture
    #   
    #
    def __init__(self, 
                 capture_path,
                 fifo_out,
                 fps = 15,
                 fps_percent = 40,
                 resolution = [640, 480],
                 event_time = 3):

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

            destroyAllWindows()

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
        #imshow('frame', frame)
        #waitKey(100)

        # retorna o frame capturado
        return frame

class VideoHandler():

    ## @brief Instanciador da classe VideoHandler
    #   
    def __init__(self,
                 name,
                 fifo_in,
                 fps,
                 resolution,
                 video_duration):

        # Aloca o nome para os arquivos de vídeo e de texto
        self.name = name
        print(f'name = {self.name}')

        # Aloca a fila de entrada dos frames
        self.fifo = fifo_in

        # Aloca a velocidade de escrita do vídeo
        self.fps = int(fps)
        print(f'fps = {self.fps}')

        # Aloca a resolução dos frames
        self.resolution = resolution
        print(f'resolution = ({self.resolution[0]}, {self.resolution[1]})')

        # Calcula a quantidade de frames dentro do evento capturado
        self.frame_by_event = int(fps * video_duration)
        print(f'Total frames = {self.frame_by_event}')

    ## @brief
    #
    def run(self):

        print('Iniciando VideoHandler.putInMP4 ...')

        '''
        # Define o formato do vídeo como mp4
        fourcc = VideoWriter_fourcc(*'mp4v')
        print(f'fourcc = {fourcc}')

        # Abre o arquivo para salvar os frames
        writer = VideoWriter(f'{self.name}.mp4', 
                             fourcc,  
                             self.fps, 
                             (self.resolution[0], self.resolution[1]))

        print(f'VH: writer aberto em {self.name}.mp4 para {self.frame_by_event} frames a {self.fps} fps')
        '''

        # Abre o arquivo de texto onde os frames serão escritos
        fd = open(f'{self.name}.txt', 'w')

        print(f'VH: Descritor de arquivo fd = {fd}')

        # De acordo com o total de frames em um evento
        for i in range(self.frame_by_event):

            # Pega cada frame na fila
            frame = self.fifo.get_nowait()
            
            # Coloca o frame em escala de cinza
            frame = cvtColor(frame, COLOR_BGR2GRAY)

            # e o escreve no arquivo de vídeo
            #writer.write(frame) 

            # Apresenta o Frame na tela
            imshow('Grayframe', frame)
            waitKey(100)

            # monta um vetor de pixel do frame capturado
            pixel_vector = np.concatenate(frame)

            # Escreve cada pixel do vetor no arquivo de texto
            for pixel in pixel_vector:
                fd.write('{:08b}\n'.format(pixel))

        # Fecha as janela de apresentação dos frames
        destroyAllWindows()

        print(f'VH: Todos os frames foram gravados no arquivo {self.name}.txt')

        # Libera o objeto escritor de vídeo 
        #writer.release()
        #print('VH: Objeto Writer fechado')

        # Fecha o arquivo de texto
        fd.close()
        print('VH: Descritor de arquivo fechado. Finalizando VideoHandler')


if __name__ == "__main__":

    # Recebendo os argumentos
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("-pth", "--capture_path", required=True, 
    help="Caminho para a fonte de captura")
    ap.add_argument("-name", "--file_name", required=True, 
    help="Nome do arquivo txt que será gerado")
    args = vars(ap.parse_args())

    # Instancia uma FIFO para enfileirar os frames capturados
    # 5 segundos de um vídeo a 20% de 15 FPS
    fifo = Queue(maxsize=15*0.4*3)

    # Instancia um obejto FrameCapture
    fc = FrameCapture(args["capture_path"], fifo)

    # Instancia um objeto VideoHandler
    vh = VideoHandler(args["file_name"], fifo, 15*0.4, [640, 480], 3)

    # Inicia as operações do objeto FrameCapture
    fc.start()

    # Aguarda enquanto o capturador de frames insere as imagens na fila
    while not fifo.full():
        print(f' fifo_size = {fifo.qsize()}')
        sleep(1)

    # Quando a fila estiver cheia
    if fifo.full():

        print(f' fifo_size = {fifo.qsize()}')

        print("Fila cheia")

        # Para o capturador de frames
        fc.stop()

        # Monta o vídeo com os frames capturados em um video mp4
        # E escreve pixel a pixels num arquivo txt
        vh.run()

    


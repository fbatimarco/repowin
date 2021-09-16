import tkinter as tk
import model
import pdb
import serial
from time import sleep
import logging
import sys
import os
from tkinter import *
from tkinter.ttk import *

from PIL import Image, ImageTk

sys.setrecursionlimit(10**6)

logging.basicConfig(format='%(asctime)s - %(levelname)s: %(message)s', filename='/home/pi/caixa-magica-teste-HW/funcionamento_hardwares.log', filemode='a', level=logging.INFO)
import json
#CONFIG = None
#with open('/home/pi/caixa-magica/core/config.json') as json_data:
#    CONFIG = json.load(json_data)

fonteTitulo = ('Arial', '20', 'bold')
fonteTexto = ('Arial', '18')
fonteTeste = ('Arial', '14')


class SeaofBTCapp(tk.Tk):

    def __init__(self, *args, **kwargs):
        
        tk.Tk.__init__(self, *args, **kwargs)
        container = tk.Frame(self)

        self.attributes('-fullscreen', True)

        container.pack(side="top", fill="both", expand = True)

        container.grid_rowconfigure(0, weight=1)
        container.grid_columnconfigure(0, weight=1)

        self.frames = {}

        for F in (StartPage, PageOne, PageTwo, PageThree, PageFour, PageFive, PageSix, PageSeven, PageEight, PageNine, PageTen):
        # for F in (StartPage, PageOne, PageTwo, PageThree):

            frame = F(container, self)

            self.frames[F] = frame

            frame.grid(row=0, column=0, sticky="nsew")

        # self.show_frame(PageOne)
        self.show_frame(StartPage)

    def show_frame(self, cont):

        frame = self.frames[cont]
        frame.tkraise()

        
class StartPage(tk.Frame):

    def def_teste():
        print("Hello world")

    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)

        self.pady = '20'

        self.primeiroContainer = tk.Frame(self)
        self.primeiroContainer['pady'] = self.pady
        self.primeiroContainer['bg'] = 'white'
        self.primeiroContainer.pack()

        # self.segundoContainer = tk.Frame(self)
        # self.segundoContainer['pady'] = self.pady
        # self.segundoContainer.pack()

        # self.terceiroContainer = tk.Frame(self)
        # self.terceiroContainer['pady'] = self.pady
        # self.terceiroContainer.pack()

        # self.quartoContainer = tk.Frame(self)
        # self.quartoContainer['pady'] = self.pady
        # self.quartoContainer.pack()

        # self.quintoContainer = tk.Frame(self)
        # self.quintoContainer['pady'] = self.pady
        # self.quintoContainer.pack()

        # self.sextoContainer = tk.Frame(self)
        # self.sextoContainer['pady'] = self.pady
        # self.sextoContainer.pack()  

        # self.setimoContainer = tk.Frame(self)
        # self.setimoContainer['pady'] = self.pady
        # self.setimoContainer.pack()

        # self.oitavoContainer = tk.Frame(self)
        # self.oitavoContainer['pady'] = self.pady
        # self.oitavoContainer.pack()  

        # self.nonoContainer = tk.Frame(self)
        # self.nonoContainer['pady'] = self.pady
        # self.nonoContainer.pack()

        # self.decimoContainer = tk.Frame(self)
        # self.decimoContainer['pady'] = self.pady
        # self.decimoContainer.pack()      
        
        self.titulo_0 = tk.Label(self.primeiroContainer, text="MENU DE TESTES")
        self.titulo_0['width'] = 35
        self.titulo_0['font'] = ('Arial', '25', 'bold')
        self.titulo_0['bg'] = 'white'
        self.titulo_0['justify'] = tk.CENTER
        self.titulo_0.pack(pady=12)

        self.button2 = tk.Button(self.primeiroContainer, text="GPS",
                            command=lambda: controller.show_frame(PageTwo))
        self.button2['font'] = fonteTexto
        self.button2['width'] = 14
        self.button2['height'] = 1
        self.button2.pack(pady=12)

        self.button3 = tk.Button(self.primeiroContainer, text="Camera", command=lambda: controller.show_frame(PageThree))
        self.button3['font'] = fonteTexto
        self.button3['width'] = 14
        self.button3['height'] = 1
        self.button3.pack(pady=12)
        
        self.button4 = tk.Button(self.primeiroContainer, text="Buzzer", command=lambda: controller.show_frame(PageFour))
        self.button4['font'] = fonteTexto
        self.button4['width'] = 14
        self.button4['height'] = 1
        self.button4.pack(pady=12)

        #keylogger = Tk()
        self.bind('b',self.def_teste)

        self.button5 = tk.Button(self.primeiroContainer, text="Botoeira",
                            command=lambda: controller.show_frame(PageFive)) 
        self.button5['font'] = fonteTexto
        self.button5['width'] = 14  
        self.button5['height'] = 1
        self.button5.pack(pady=12)

        self.button6 = tk.Button(self.primeiroContainer, text="Internet",
                            command=lambda: controller.show_frame(PageSix)) 
        self.button6['font'] = fonteTexto
        self.button6['width'] = 14  
        self.button6['height'] = 1  
        self.button6.pack(pady=12)

        self.button7 = tk.Button(self.primeiroContainer, text="Catraca",
                            command=lambda: controller.show_frame(PageSeven)) 
        self.button7['font'] = fonteTexto
        self.button7['width'] = 14 
        self.button7['height'] = 1
        self.button7.pack(pady=12)

        self.button8 = tk.Button(self.primeiroContainer, text="RFID",
                            command=lambda: controller.show_frame(PageEight)) 
        self.button8['font'] = fonteTexto
        self.button8['width'] = 14 
        self.button8['height'] = 1
        self.button8.pack(pady=12)

        self.button9 = tk.Button(self.primeiroContainer, text="Sonar",
                            command=lambda: controller.show_frame(PageNine))
        self.button9['font'] = fonteTexto
        self.button9['width'] = 14 
        self.button9['height'] = 1 
        self.button9.pack(pady=12)

        self.button10 = tk.Button(self.primeiroContainer, text="Sair",
                            command=self.destruir)
        self.button10['font'] = fonteTexto
        self.button10['width'] = 14 
        self.button10['height'] = 1 
        self.button10.pack(pady=(40, 20))

    def destruir(self):
        app.destroy()


class PageOne(tk.Frame): # Touch Screen

    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)

        self.pady = '20'
        self.width = '12'

        self.primeiroContainer = tk.Frame(self)
        self.primeiroContainer['pady'] = self.pady
        self.primeiroContainer.pack()

        self.segundoContainer = tk.Frame(self)
        self.segundoContainer['pady'] = self.pady
        self.segundoContainer.pack()

        self.terceiroContainer = tk.Frame(self)
        self.terceiroContainer['pady'] = self.pady
        self.terceiroContainer.pack()

        self.quartoContainer = tk.Frame(self)
        self.quartoContainer['pady'] = self.pady
        self.quartoContainer.pack()

        self.quintoContainer = tk.Frame(self)
        self.quintoContainer['pady'] = self.pady
        self.quintoContainer.pack()

        self.sextoContainer = tk.Frame(self)
        self.sextoContainer['pady'] = self.pady
        self.sextoContainer.pack()

        self.setimoContainer = tk.Frame(self)
        self.setimoContainer['pady'] = self.pady
        self.setimoContainer.pack()

        self.oitavoContainer = tk.Frame(self)
        self.oitavoContainer['pady'] = self.pady
        self.oitavoContainer.pack(pady=(50, 20))

        self.nonoContainer = tk.Frame(self)
        self.nonoContainer.pack(padx=(45, 45), anchor=tk.E)

        self.titulo_0 = tk.Label(self.primeiroContainer)
        self.titulo_0['text'] = "Teste de Hardware"
        self.titulo_0["font"] = fonteTitulo
        self.titulo_0.pack()

        self.titulo_1 = tk.Message(self.segundoContainer)
        self.titulo_1['text'] = 'Seja bem vindo aos testes de hardware da Caixa Mágica.\n\nPara iniciar, pressione o botão abaixo para verificar se o touch screen do display está funcionando corretamente.'
        self.titulo_1["font"] = fonteTexto
        self.titulo_1['width'] = '420'
        self.titulo_1['justify'] = tk.CENTER
        self.titulo_1.pack()

        self.botao_1 = tk.Button(self.terceiroContainer)
        self.botao_1["text"] = "Testar"
        self.botao_1["font"] = fonteTexto
        self.botao_1['command'] = lambda: [self.touch(), self.ativar_botao()]
        self.botao_1['width'] = self.width
        self.botao_1.pack()

        self.msg_1 = tk.Label(self.quartoContainer)
        self.msg_1['text'] = ''
        self.msg_1["font"] = fonteTeste
        self.msg_1.pack()

        self.titulo_2 = tk.Message(self.quintoContainer)
        self.titulo_2['text'] = ''
        self.titulo_2["font"] = fonteTexto
        self.titulo_2['width'] = '420'
        self.titulo_2['justify'] = tk.CENTER
        self.titulo_2.pack()

        # self.botao_2 = tk.Button(self.sextoContainer)
        # self.botao_2["text"] = "Testar"
        # self.botao_2["font"] = fonteTexto
        # self.botao_2['command'] = None
        # self.botao_2.pack()

        self.msg_2 = tk.Label(self.setimoContainer)
        self.msg_2['text'] = ''
        self.msg_2["font"] = fonteTeste
        self.msg_2.pack()

        self.titulo_3 = tk.Label(self.oitavoContainer)
        self.titulo_3['text'] = 'Se funcionou, avance.'
        self.titulo_3["font"] = fonteTexto
        self.titulo_3.pack()

        self.botao_3 = tk.Button(self.nonoContainer)
        self.botao_3["text"] = "Próximo"
        self.botao_3["font"] = fonteTexto
        self.botao_3['command'] = lambda: controller.show_frame(PageTwo)
        self.botao_3['width'] = self.width
        # self.botao_3['state'] = tk.DISABLED
        self.botao_3['state'] = tk.NORMAL
        self.botao_3.pack(padx=10, side=tk.RIGHT)


    def touch(self):
        self.msg_1['fg'] = 'green'
        self.msg_1['text'] = 'TESTE OK'

        logging.info('Touch Screen funcionou')
    
    def ativar_botao(self):
        self.botao_3['state'] = tk.NORMAL


class PageTwo(tk.Frame): # GPS

    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)

        self.pady = '20'
        self.width = '12'

        self.primeiroContainer = tk.Frame(self)
        self.primeiroContainer['pady'] = self.pady
        self.primeiroContainer.pack()

        self.segundoContainer = tk.Frame(self)
        self.segundoContainer['pady'] = self.pady
        self.segundoContainer.pack()

        self.terceiroContainer = tk.Frame(self)
        self.terceiroContainer['pady'] = self.pady
        self.terceiroContainer.pack()

        self.quartoContainer = tk.Frame(self)
        self.quartoContainer['pady'] = self.pady
        self.quartoContainer.pack()

        self.quintoContainer = tk.Frame(self)
        self.quintoContainer['pady'] = self.pady
        self.quintoContainer.pack()

        self.sextoContainer = tk.Frame(self)
        self.sextoContainer['pady'] = self.pady
        self.sextoContainer.pack()

        self.setimoContainer = tk.Frame(self)
        self.setimoContainer['pady'] = self.pady
        self.setimoContainer.pack()

        self.oitavoContainer = tk.Frame(self)
        self.oitavoContainer.pack(pady=(115, 20))

        self.nonoContainer = tk.Frame(self)
        self.nonoContainer['pady'] = self.pady
        self.nonoContainer.pack(padx=(45, 45), anchor=tk.E)

        self.titulo_0 = tk.Label(self.primeiroContainer)
        self.titulo_0['text'] = "Teste do GPS"
        self.titulo_0["font"] = fonteTitulo
        self.titulo_0.pack()

        self.titulo_1 = tk.Message(self.segundoContainer)
        self.titulo_1['text'] = 'Pressione o botão para testar a comunicação serial e o sinal do GPS:'
        self.titulo_1['width'] = '420'
        self.titulo_1['justify'] = tk.CENTER
        self.titulo_1["font"] = fonteTexto
        self.titulo_1.pack()

        self.botao_1 = tk.Button(self.terceiroContainer)
        self.botao_1["text"] = "Testar"
        self.botao_1["font"] = fonteTexto
        self.botao_1['command'] = lambda:[self.gps_1(), self.gps_2(), self.ativar_botao()]
        self.botao_1['width'] = self.width
        self.botao_1.pack()

        self.titulo_2 = tk.Label(self.quartoContainer)
        self.titulo_2['text'] = "Comunicação serial:"
        self.titulo_2["font"] = fonteTexto
        self.titulo_2.pack()

        self.msg_1 = tk.Label(self.quintoContainer)
        self.msg_1['text'] = ''
        self.msg_1["font"] = fonteTeste
        self.msg_1.pack()

        self.titulo_extra = tk.Label(self.sextoContainer)
        self.titulo_extra["text"] = "Sinal de localização:"
        self.titulo_extra["font"] = fonteTexto
        self.titulo_extra.pack()

        self.msg_2 = tk.Label(self.setimoContainer)
        self.msg_2['text'] = ''
        self.msg_2["font"] = fonteTeste
        self.msg_2.pack()

        self.titulo_3 = tk.Label(self.oitavoContainer)
        self.titulo_3['text'] = 'Se funcionou, avance'
        self.titulo_3["font"] = fonteTexto
        self.titulo_3.pack()

        self.botao_3 = tk.Button(self.nonoContainer)
        self.botao_3["text"] = "Próximo"
        self.botao_3["font"] = fonteTexto
        self.botao_3['command'] = lambda: controller.show_frame(PageThree)
        #self.botao_3['command'] = lambda: controller.show_frame(PageFive)
        self.botao_3['width'] = self.width
        self.botao_3['state'] = tk.NORMAL
        self.botao_3.pack(padx=10, side=tk.RIGHT)

        self.botao_4 = tk.Button(self.nonoContainer)
        self.botao_4["text"] = "Voltar"
        self.botao_4["font"] = fonteTexto
        # self.botao_4['command'] = lambda: controller.show_frame(PageOne)
        self.botao_4['command'] = lambda: controller.show_frame(StartPage)
        self.botao_4['width'] = self.width
        self.botao_4['state'] = tk.NORMAL
        self.botao_4.pack(padx=10, side=tk.RIGHT)

    def gps_1(self):
        resultado = model.teste_GPS_comm()
        if resultado == True:
            self.msg_1['fg'] = 'green'
            self.msg_1['text'] = 'GPS CONECTADO'
        else:
            self.msg_1['fg'] = 'red'
            self.msg_1['text'] = 'GPS DESCONECTADO'

        logging.info(f'Conexao serial do GPS: {resultado}')

    def gps_2(self):
        resultado = model.teste_GPS_loc()
        if resultado != None:
            self.msg_2['fg'] = 'green'
            self.msg_2['text'] = str(resultado)
        else:
            self.msg_2['fg'] = 'black'
            self.msg_2['text'] = str(resultado)

        logging.info(f'Localizacao do GPS: {resultado}')        
    
    def ativar_botao(self):
        self.botao_3['state'] = tk.NORMAL


class PageThree(tk.Frame): # Câmera

    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)

        self.pady = '20'
        self.width = '12'

        self.primeiroContainer = tk.Frame(self)
        self.primeiroContainer['pady'] = self.pady
        self.primeiroContainer.pack()

        self.segundoContainer = tk.Frame(self)
        self.segundoContainer['pady'] = self.pady
        self.segundoContainer.pack()

        self.terceiroContainer = tk.Frame(self)
        self.terceiroContainer['pady'] = self.pady
        self.terceiroContainer.pack()

        self.quartoContainer = tk.Frame(self)
        self.quartoContainer['pady'] = self.pady
        self.quartoContainer.pack()

        self.quintoContainer = tk.Frame(self)
        self.quintoContainer['pady'] = self.pady
        self.quintoContainer.pack()

        self.sextoContainer = tk.Frame(self)
        self.sextoContainer['pady'] = self.pady
        self.sextoContainer.pack()

        self.setimoContainer = tk.Frame(self)
        self.setimoContainer['pady'] = self.pady
        self.setimoContainer.pack()

        self.oitavoContainer = tk.Frame(self)
        self.oitavoContainer.pack(pady=(185, 25))

        self.nonoContainer = tk.Frame(self)
        self.nonoContainer['pady'] = self.pady
        self.nonoContainer.pack(padx=(45, 45), anchor=tk.E)

        self.titulo_0 = tk.Label(self.primeiroContainer)
        self.titulo_0['text'] = "Teste de Câmera"
        self.titulo_0["font"] = fonteTitulo
        self.titulo_0.pack()

        self.titulo_1 = tk.Message(self.segundoContainer)
        self.titulo_1['text'] = 'Pressione o botão para testar a câmera e apresente o QR code de teste'
        self.titulo_1['width'] = '420'
        self.titulo_1['justify'] = tk.CENTER
        self.titulo_1["font"] = fonteTexto
        self.titulo_1.pack()

        self.botao_1 = tk.Button(self.terceiroContainer)
        self.botao_1["text"] = "Testar"
        self.botao_1["font"] = fonteTexto
        self.botao_1['command'] = lambda:[self.camera_1(), self.ativar_botao()]
        self.botao_1['width'] = self.width
        self.botao_1.pack()

        self.msg_1 = tk.Label(self.quartoContainer)
        self.msg_1['text'] = ''
        self.msg_1["font"] = fonteTeste
        self.msg_1.pack()

        self.titulo_2 = tk.Label(self.quintoContainer)
        self.titulo_2['text'] = ''
        self.titulo_2["font"] = fonteTexto
        self.titulo_2.pack()

        # self.botao_2 = tk.Button(self.sextoContainer)
        # self.botao_2["text"] = "Testar"
        # self.botao_2["font"] = fonteTexto
        # self.botao_2['command'] = None
        # self.botao_2.pack()

        self.msg_2 = tk.Label(self.setimoContainer)
        self.msg_2['text'] = ''
        self.msg_2["font"] = fonteTeste
        self.msg_2.pack()

        self.titulo_3 = tk.Label(self.oitavoContainer)
        self.titulo_3['text'] = 'Se funcionou, avance.'
        self.titulo_3["font"] = fonteTexto
        self.titulo_3.pack()

        self.botao_3 = tk.Button(self.nonoContainer)
        self.botao_3["text"] = "Próximo"
        self.botao_3["font"] = fonteTexto
        self.botao_3['command'] = lambda: controller.show_frame(PageFour)
        self.botao_3['width'] = self.width
        #self.botao_3['state'] = tk.DISABLED
        self.botao_3.pack(padx=10, side=tk.RIGHT)

        self.botao_4 = tk.Button(self.nonoContainer)
        self.botao_4["text"] = "Voltar"
        self.botao_4["font"] = fonteTexto
        self.botao_4['command'] = lambda: controller.show_frame(PageTwo)
        self.botao_4['width'] = self.width
        self.botao_4['state'] = tk.NORMAL
        self.botao_4.pack(padx=10, side=tk.RIGHT)


    def camera_1(self):
        resultado = model.teste_camera(10)
        if resultado == True:
            self.msg_1['fg'] = 'green'
            self.msg_1['text'] = 'TESTE OK'
        else:
            self.msg_1['fg'] = 'red'
            self.msg_1['text'] = 'Ajuste o foco da câmera e tente novamente.'
        # Quando a camera fechar, aparecer OK ou NOK
        # Colocar timeout na camera para sair caso nao consiga ler o qrcode

        logging.info(f'Conexao da Camera: {resultado}')

    def ativar_botao(self):
        self.botao_3['state'] = tk.NORMAL


class PageFour(tk.Frame): # Buzzer

    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)

        self.pady = '20'
        self.width = '12'

        self.primeiroContainer = tk.Frame(self)
        self.primeiroContainer['pady'] = self.pady
        self.primeiroContainer.pack()

        self.segundoContainer = tk.Frame(self)
        self.segundoContainer['pady'] = self.pady
        self.segundoContainer.pack()

        self.terceiroContainer = tk.Frame(self)
        self.terceiroContainer['pady'] = self.pady
        self.terceiroContainer.pack()

        self.quartoContainer = tk.Frame(self)
        self.quartoContainer['pady'] = self.pady
        self.quartoContainer.pack()

        self.quintoContainer = tk.Frame(self)
        self.quintoContainer['pady'] = self.pady
        self.quintoContainer.pack()

        self.sextoContainer = tk.Frame(self)
        self.sextoContainer['pady'] = self.pady
        self.sextoContainer.pack()

        self.setimoContainer = tk.Frame(self)
        self.setimoContainer['pady'] = self.pady
        self.setimoContainer.pack()

        self.oitavoContainer = tk.Frame(self)
        self.oitavoContainer.pack(pady=(130, 20))

        self.nonoContainer = tk.Frame(self)
        self.nonoContainer['pady'] = self.pady
        self.nonoContainer.pack(padx=(45, 45), anchor=tk.E)

        self.titulo_0 = tk.Label(self.primeiroContainer)
        self.titulo_0['text'] = "Teste de Buzzer"
        self.titulo_0["font"] = fonteTitulo
        self.titulo_0.pack()

        self.titulo_1 = tk.Message(self.segundoContainer)
        self.titulo_1['text'] = 'Pressione o botão para testar o buzzer'
        self.titulo_1['width'] = '420'
        self.titulo_1['justify'] = tk.CENTER
        self.titulo_1["font"] = fonteTexto
        self.titulo_1.pack()

        self.botao_1 = tk.Button(self.terceiroContainer)
        self.botao_1["text"] = "Testar"
        self.botao_1["font"] = fonteTexto
        self.botao_1['command'] = self.buzzer
        self.botao_1['width'] = self.width
        self.botao_1.pack()

        self.msg_1 = tk.Label(self.quartoContainer)
        self.msg_1['text'] = ''
        self.msg_1["font"] = fonteTeste
        self.msg_1.pack()

        self.titulo_2 = tk.Label(self.quintoContainer)
        self.titulo_2['text'] = ''
        self.titulo_2["font"] = fonteTexto
        self.titulo_2.pack()

        self.botao_2 = tk.Button(self.sextoContainer)
        self.botao_2["text"] = 'Não'
        self.botao_2["font"] = fonteTexto
        self.botao_2['width'] = 8
        self.botao_2['command'] = lambda: [self.buzzer_nao(), self.ativar_botao()]
        self.botao_2['state'] = tk.DISABLED
        self.botao_2.pack(padx=20, side=tk.RIGHT)

        self.botao_extra = tk.Button(self.sextoContainer)
        self.botao_extra['text'] = 'Sim'
        self.botao_extra['font'] = fonteTexto
        self.botao_extra['command'] = lambda: [self.buzzer_sim(), self.ativar_botao()]
        self.botao_extra['width'] = 8
        self.botao_extra['state'] = tk.DISABLED
        self.botao_extra.pack(padx=20, side=tk.RIGHT)

        self.msg_2 = tk.Label(self.setimoContainer)
        self.msg_2['text'] = ''
        self.msg_2["font"] = fonteTeste
        self.msg_2.pack()

        self.titulo_3 = tk.Label(self.oitavoContainer)
        self.titulo_3['text'] = 'Se funcionou, avance'
        self.titulo_3["font"] = fonteTexto
        self.titulo_3.pack()

        self.botao_3 = tk.Button(self.nonoContainer)
        self.botao_3["text"] = "Próximo"
        self.botao_3["font"] = fonteTexto
        self.botao_3['command'] = lambda: controller.show_frame(PageFive)
        self.botao_3['width'] = self.width
        self.botao_3['state'] = tk.DISABLED
        self.botao_3.pack(padx=10, side=tk.RIGHT)

        self.botao_4 = tk.Button(self.nonoContainer)
        self.botao_4["text"] = "Voltar"
        self.botao_4["font"] = fonteTexto
        self.botao_4['command'] = lambda: controller.show_frame(PageThree)
        self.botao_4['width'] = self.width
        self.botao_4['state'] = tk.NORMAL
        self.botao_4.pack(padx=10, side=tk.RIGHT)


    def buzzer(self):
        model.teste_buzzer()
        sleep(0.5)
        self.msg_1['text'] = 'Foi possível ouvir o Buzzer?'

        self.botao_2['state'] = tk.NORMAL
        self.botao_extra['state'] = tk.NORMAL

    def buzzer_sim(self):
        self.msg_2['fg'] = 'green'
        self.msg_2['text'] = 'TESTE OK'

        self.botao_2['state'] = tk.DISABLED

        logging.info(f'Buzzer funcionou')

    def buzzer_nao(self):
        self.msg_2['fg'] = 'red'
        self.msg_2['text'] = 'TESTE FALHOU'

        self.botao_extra['state'] = tk.DISABLED

        logging.info(f'Buzzer nao funcionou')

    def ativar_botao(self):
        self.botao_3['state'] = tk.NORMAL


class PageFive(tk.Frame): # Botoeira

    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)

        self.pady = '20'
        self.width = '12'

        self.primeiroContainer = tk.Frame(self)
        self.primeiroContainer['pady'] = self.pady
        self.primeiroContainer.pack()

        self.segundoContainer = tk.Frame(self)
        self.segundoContainer['pady'] = self.pady
        self.segundoContainer.pack()

        self.terceiroContainer = tk.Frame(self)
        self.terceiroContainer['pady'] = self.pady
        self.terceiroContainer.pack()

        self.quartoContainer = tk.Frame(self)
        self.quartoContainer['pady'] = self.pady
        self.quartoContainer.pack()

        self.quintoContainer = tk.Frame(self)
        self.quintoContainer['pady'] = self.pady
        self.quintoContainer.pack()

        self.sextoContainer = tk.Frame(self)
        self.sextoContainer['pady'] = self.pady
        self.sextoContainer.pack()

        self.setimoContainer = tk.Frame(self)
        self.setimoContainer['pady'] = self.pady
        self.setimoContainer.pack()

        self.oitavoContainer = tk.Frame(self)
        self.oitavoContainer.pack(pady=(110, 20))

        self.nonoContainer = tk.Frame(self)
        self.nonoContainer['pady'] = self.pady
        self.nonoContainer.pack(padx=(45, 45), anchor=tk.E)

        self.titulo_0 = tk.Label(self.primeiroContainer)
        self.titulo_0['text'] = "Teste de Botoeira"
        self.titulo_0["font"] = fonteTitulo
        self.titulo_0.pack()

        self.titulo_1 = tk.Message(self.segundoContainer)
        self.titulo_1['text'] = 'Pressione cada botão 3 vezes.\nBotão de (G)ratuidade'
        self.titulo_1['width'] = '420'
        self.titulo_1['justify'] = tk.CENTER
        self.titulo_1["font"] = fonteTexto
        self.titulo_1.pack()

        self.botao_1 = tk.Button(self.terceiroContainer)
        self.botao_1["text"] = "Testar"
        self.botao_1["font"] = fonteTexto
        self.botao_1['command'] = self.botoeira_1
        self.botao_1['width'] = self.width
        self.botao_1.pack()

        self.msg_1 = tk.Label(self.quartoContainer)
        self.msg_1['text'] = ''
        self.msg_1["font"] = fonteTeste
        self.msg_1.pack()

        self.titulo_2 = tk.Label(self.quintoContainer)
        self.titulo_2['text'] = 'Botão de (P)agante'
        self.titulo_2["font"] = fonteTexto
        self.titulo_2.pack()

        self.botao_2 = tk.Button(self.sextoContainer)
        self.botao_2["text"] = "Testar"
        self.botao_2["font"] = fonteTexto
        self.botao_2['command'] = self.botoeira_2
        self.botao_2['width'] = self.width
        self.botao_2.pack()

        self.msg_2 = tk.Label(self.setimoContainer)
        self.msg_2['text'] = ''
        self.msg_2["font"] = fonteTeste
        self.msg_2.pack()

        self.titulo_3 = tk.Label(self.oitavoContainer)
        self.titulo_3['text'] = 'Se funcionou, avance.'
        self.titulo_3["font"] = fonteTexto
        self.titulo_3.pack()

        self.botao_3 = tk.Button(self.nonoContainer)
        self.botao_3["text"] = "Próximo"
        self.botao_3["font"] = fonteTexto
        self.botao_3['command'] = lambda: controller.show_frame(PageSix)
        self.botao_3['width'] = self.width
        self.botao_3.pack(padx=10, side=tk.RIGHT)

        self.botao_4 = tk.Button(self.nonoContainer)
        self.botao_4["text"] = "Voltar"
        self.botao_4["font"] = fonteTexto
        # self.botao_4['command'] = lambda: controller.show_frame(PageFour)
        self.botao_4['command'] = lambda: controller.show_frame(StartPage)
        self.botao_4['width'] = self.width
        self.botao_4['state'] = tk.NORMAL
        self.botao_4.pack(padx=10, side=tk.RIGHT)

    def botoeira_1(self):
        resultado = model.teste_bot_BG()
        if resultado == True:
            self.msg_1['fg'] = 'green'
            self.msg_1['text'] = 'TESTE OK'
        else:
            self.msg_1['fg'] = 'red'
            self.msg_1['text'] = 'TESTE FALHOU'

        logging.info(f'Botoeira Gratuidade: {resultado}')
            
    def botoeira_2(self):
        resultado = model.teste_bot_BP()
        if resultado == True:
            self.msg_2['fg'] = 'green'
            self.msg_2['text'] = 'TESTE OK'
        else:
            self.msg_2['fg'] = 'red'
            self.msg_2['text'] = 'TESTE FALHOU'

        logging.info(f'Botoeira Pagante: {resultado}')


class PageSix(tk.Frame): # Internet

    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)

        self.pady = '20'
        self.width = '12'

        self.primeiroContainer = tk.Frame(self)
        self.primeiroContainer['pady'] = self.pady
        self.primeiroContainer.pack()

        self.segundoContainer = tk.Frame(self)
        self.segundoContainer['pady'] = self.pady
        self.segundoContainer.pack()

        self.terceiroContainer = tk.Frame(self)
        self.terceiroContainer['pady'] = self.pady
        self.terceiroContainer.pack()

        self.quartoContainer = tk.Frame(self)
        self.quartoContainer['pady'] = self.pady
        self.quartoContainer.pack()

        self.quintoContainer = tk.Frame(self)
        self.quintoContainer['pady'] = self.pady
        self.quintoContainer.pack()

        self.sextoContainer = tk.Frame(self)
        self.sextoContainer['pady'] = self.pady
        self.sextoContainer.pack()

        self.setimoContainer = tk.Frame(self)
        self.setimoContainer['pady'] = self.pady
        self.setimoContainer.pack()

        self.oitavoContainer = tk.Frame(self)
        self.oitavoContainer.pack(pady=(115, 20))

        self.nonoContainer = tk.Frame(self)
        self.nonoContainer['pady'] = self.pady
        self.nonoContainer.pack(padx=(45, 45), anchor=tk.E)

        self.titulo_0 = tk.Label(self.primeiroContainer)
        self.titulo_0['text'] = "Teste de Internet"
        self.titulo_0["font"] = fonteTitulo
        self.titulo_0.pack()

        self.titulo_1 = tk.Message(self.segundoContainer)
        self.titulo_1['text'] = 'Pressione o botão para testar a comunicação de internet:'
        self.titulo_1['width'] = '420'
        self.titulo_1['justify'] = tk.CENTER
        self.titulo_1["font"] = fonteTexto
        self.titulo_1.pack()

        self.botao_1 = tk.Button(self.terceiroContainer)
        self.botao_1["text"] = "Testar"
        self.botao_1["font"] = fonteTexto
        self.botao_1['command'] = lambda:[self.internet_1(), self.internet_2(), self.ativar_botao()]
        self.botao_1['width'] = self.width
        self.botao_1.pack()

        self.titulo_2 = tk.Label(self.quartoContainer)
        self.titulo_2['text'] = 'Conexão Wifi:'
        self.titulo_2["font"] = fonteTexto
        self.titulo_2.pack()
        
        self.msg_1 = tk.Label(self.quintoContainer)
        self.msg_1['text'] = ''
        self.msg_1["font"] = fonteTeste
        self.msg_1.pack()

        self.titulo_extra = tk.Label(self.sextoContainer)
        self.titulo_extra["text"] = "Conexão com a internet:"
        self.titulo_extra["font"] = fonteTexto
        self.titulo_extra.pack()

        self.msg_2 = tk.Label(self.setimoContainer)
        self.msg_2['text'] = ''
        self.msg_2["font"] = fonteTeste
        self.msg_2.pack()

        self.titulo_3 = tk.Label(self.oitavoContainer)
        self.titulo_3['text'] = 'Se funcionou, avance'
        self.titulo_3["font"] = fonteTexto
        self.titulo_3.pack()

        self.botao_3 = tk.Button(self.nonoContainer)
        self.botao_3["text"] = "Próximo"
        self.botao_3["font"] = fonteTexto
        self.botao_3['command'] = lambda: controller.show_frame(PageSeven)
        # self.botao_3['command'] = lambda: controller.show_frame(PageEight)
        self.botao_3['width'] = self.width
        # self.botao_3['state'] = tk.DISABLED
        self.botao_3['state'] = tk.NORMAL
        self.botao_3.pack(padx=10, side=tk.RIGHT)

        self.botao_4 = tk.Button(self.nonoContainer)
        self.botao_4["text"] = "Voltar"
        self.botao_4["font"] = fonteTexto
        # self.botao_4['command'] = lambda: controller.show_frame(PageFive)
        self.botao_4['command'] = lambda: controller.show_frame(StartPage)
        self.botao_4['width'] = self.width
        self.botao_4['state'] = tk.NORMAL
        self.botao_4.pack(padx=10, side=tk.RIGHT)

    def internet_1(self):
        resultado = model.teste_wifi()
        if resultado == True:
            self.msg_1['fg'] = 'green'
            self.msg_1['text'] = 'TESTE OK'
        else:
            self.msg_1['fg'] = 'red'
            self.msg_1['text'] = 'TESTE FALHOU'

        logging.info(f'Conexao Wifi: {resultado}')

    def internet_2(self):
        resultado = model.teste_internet()
        if resultado == True:
            self.msg_2['fg'] = 'green'
            self.msg_2['text'] = 'TESTE OK'
        else:
            self.msg_2['fg'] = 'red'
            self.msg_2['text'] = 'TESTE FALHOU'

        logging.info(f'Conexao com a internet: {resultado}')

    def ativar_botao(self):
        self.botao_3['state'] = tk.NORMAL


class PageSeven(tk.Frame): # Catraca

    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)

        self.pady = '20'
        self.width = '12'

        self.primeiroContainer = tk.Frame(self)
        self.primeiroContainer['pady'] = self.pady
        self.primeiroContainer.pack()

        self.segundoContainer = tk.Frame(self)
        self.segundoContainer['pady'] = self.pady
        self.segundoContainer.pack()

        self.terceiroContainer = tk.Frame(self)
        self.terceiroContainer['pady'] = self.pady
        self.terceiroContainer.pack()

        self.quartoContainer = tk.Frame(self)
        self.quartoContainer['pady'] = self.pady
        self.quartoContainer.pack()

        self.quintoContainer = tk.Frame(self)
        self.quintoContainer['pady'] = self.pady
        self.quintoContainer.pack()

        self.sextoContainer = tk.Frame(self)
        self.sextoContainer['pady'] = self.pady
        self.sextoContainer.pack()

        self.setimoContainer = tk.Frame(self)
        self.setimoContainer['pady'] = self.pady
        self.setimoContainer.pack()

        self.oitavoContainer = tk.Frame(self)
        self.oitavoContainer.pack(pady=(175, 20))
        self.oitavoContainer.pack()

        self.nonoContainer = tk.Frame(self)
        self.nonoContainer['pady'] = self.pady
        self.nonoContainer.pack(padx=(45, 45), anchor=tk.E)

        self.titulo_0 = tk.Label(self.primeiroContainer)
        self.titulo_0['text'] = "Teste de Catraca"
        self.titulo_0["font"] = fonteTitulo
        self.titulo_0.pack()

        self.titulo_1 = tk.Message(self.segundoContainer)
        self.titulo_1['text'] = 'Pressione o botão para testar a catraca'
        self.titulo_1['width'] = '420'
        self.titulo_1['justify'] = tk.CENTER
        self.titulo_1["font"] = fonteTexto
        self.titulo_1.pack()

        self.botao_1 = tk.Button(self.terceiroContainer)
        self.botao_1["text"] = "Testar"
        self.botao_1["font"] = fonteTexto
        # self.botao_1['command'] = lambda:[self.catraca(), self.ativar_botao()]
        self.botao_1['command'] = self.catraca
        self.botao_1['width'] = self.width
        self.botao_1.pack()

        self.msg_1 = tk.Label(self.quartoContainer)
        self.msg_1['text'] = ''
        self.msg_1["font"] = fonteTeste
        self.msg_1.pack()

        self.titulo_2 = tk.Label(self.quintoContainer)
        self.titulo_2['text'] = ''
        self.titulo_2["font"] = fonteTexto
        self.titulo_2.pack()

        # self.botao_2 = tk.Button(self.sextoContainer)
        # self.botao_2["text"] = "Testar"
        # self.botao_2["font"] = fonteTexto
        # self.botao_2['command'] = None
        # self.botao_2.pack()

        self.msg_2 = tk.Label(self.setimoContainer)
        self.msg_2['text'] = ''
        self.msg_2["font"] = fonteTeste
        self.msg_2.pack()

        self.titulo_3 = tk.Message(self.oitavoContainer)
        self.titulo_3['text'] = ''
        self.titulo_3["font"] = fonteTexto
        self.titulo_3['width'] = '420'
        self.titulo_3['justify'] = tk.CENTER
        self.titulo_3.pack()

        self.botao_3 = tk.Button(self.nonoContainer)
        self.botao_3["text"] = "Próximo"
        self.botao_3["font"] = fonteTexto
        self.botao_3['command'] = lambda: controller.show_frame(PageEight)
        self.botao_3['width'] = self.width
        # self.botao_3['state'] = tk.DISABLED
        self.botao_3['state'] = tk.NORMAL
        self.botao_3.pack(padx=10, side=tk.RIGHT)

        self.botao_4 = tk.Button(self.nonoContainer)
        self.botao_4["text"] = "Voltar"
        self.botao_4["font"] = fonteTexto
        # self.botao_4['command'] = lambda: controller.show_frame(PageSix)
        self.botao_4['command'] = lambda: controller.show_frame(StartPage)
        self.botao_4['width'] = self.width
        self.botao_4['state'] = tk.NORMAL
        self.botao_4.pack(padx=10, side=tk.RIGHT)

    def catraca(self):
        resultado = model.teste_catraca()
        if resultado == True:
            self.msg_1['fg'] = 'green'
            self.msg_1['text'] = 'TESTE OK'
        else:
            self.msg_1['fg'] = 'red'
            self.msg_1['text'] = 'TESTE FALHOU'

        logging.info(f'Catraca: {resultado}')

    def ativar_botao(self):
        self.botao_3['state'] = tk.NORMAL


class PageEight(tk.Frame): # RFID

    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)

        self.pady = '20'
        self.width = '12'

        self.primeiroContainer = tk.Frame(self)
        self.primeiroContainer['pady'] = self.pady
        self.primeiroContainer.pack()

        self.segundoContainer = tk.Frame(self)
        self.segundoContainer['pady'] = self.pady
        self.segundoContainer.pack()

        self.terceiroContainer = tk.Frame(self)
        self.terceiroContainer['pady'] = self.pady
        self.terceiroContainer.pack()

        self.quartoContainer = tk.Frame(self)
        self.quartoContainer['pady'] = self.pady
        self.quartoContainer.pack()

        self.quintoContainer = tk.Frame(self)
        self.quintoContainer['pady'] = self.pady
        self.quintoContainer.pack()

        self.sextoContainer = tk.Frame(self)
        self.sextoContainer['pady'] = self.pady
        self.sextoContainer.pack()

        self.setimoContainer = tk.Frame(self)
        self.setimoContainer['pady'] = self.pady
        self.setimoContainer.pack()

        self.oitavoContainer = tk.Frame(self)
        self.oitavoContainer.pack(pady=(175, 20))
        self.oitavoContainer.pack()

        self.nonoContainer = tk.Frame(self)
        self.nonoContainer['pady'] = self.pady
        self.nonoContainer.pack(padx=(45, 45), anchor=tk.E)

        self.titulo_0 = tk.Label(self.primeiroContainer)
        self.titulo_0['text'] = "Teste de RFID"
        self.titulo_0["font"] = fonteTitulo
        self.titulo_0.pack()

        self.titulo_1 = tk.Message(self.segundoContainer)
        self.titulo_1['text'] = 'Posicione o cartão de testes e pressione o botão para testar'
        self.titulo_1['width'] = '420'
        self.titulo_1['justify'] = tk.CENTER
        self.titulo_1["font"] = fonteTexto
        self.titulo_1.pack()

        self.botao_1 = tk.Button(self.terceiroContainer)
        self.botao_1["text"] = "Testar"
        self.botao_1["font"] = fonteTexto
        self.botao_1['command'] = lambda:[self.rfid(), self.ativar_botao()]
        self.botao_1['width'] = self.width
        self.botao_1.pack()

        self.msg_1 = tk.Message(self.quartoContainer)
        self.msg_1['text'] = ''
        self.msg_1['width'] = '420'
        self.msg_1['justify'] = tk.CENTER
        self.msg_1["font"] = fonteTeste
        self.msg_1.pack()

        self.titulo_2 = tk.Label(self.quintoContainer)
        self.titulo_2['text'] = ''
        self.titulo_2["font"] = fonteTexto
        self.titulo_2.pack()

        # self.botao_2 = tk.Button(self.sextoContainer)
        # self.botao_2["text"] = "Testar"
        # self.botao_2["font"] = fonteTexto
        # self.botao_2['command'] = None
        # self.botao_2.pack()

        self.msg_2 = tk.Label(self.setimoContainer)
        self.msg_2['text'] = ''
        self.msg_2["font"] = fonteTeste
        self.msg_2.pack()

        self.titulo_3 = tk.Message(self.oitavoContainer)
        self.titulo_3['text'] = 'Avance para gerar o código de barras e cadastar a Caixa Mágica no sistema.'
        self.titulo_3["font"] = fonteTexto
        self.titulo_3['width'] = '420'
        self.titulo_3['justify'] = tk.CENTER
        self.titulo_3.pack()

        self.botao_3 = tk.Button(self.nonoContainer)
        self.botao_3["text"] = "Próximo"
        self.botao_3["font"] = fonteTexto
        self.botao_3['command'] = lambda: controller.show_frame(PageNine)
        self.botao_3['width'] = self.width
        # self.botao_3['state'] = tk.DISABLED
        self.botao_3['state'] = tk.NORMAL
        self.botao_3.pack(padx=10, side=tk.RIGHT)

        self.botao_4 = tk.Button(self.nonoContainer)
        self.botao_4["text"] = "Voltar"
        self.botao_4["font"] = fonteTexto
        # self.botao_4['command'] = lambda: controller.show_frame(PageSeven)
        self.botao_4['command'] = lambda: controller.show_frame(StartPage)
        self.botao_4['width'] = self.width
        self.botao_4['state'] = tk.NORMAL
        self.botao_4.pack(padx=10, side=tk.RIGHT)

    def rfid(self):
        resultado = model.teste_rfid()
        if resultado == 'serial':
            resultado = 'Erro com conexao com modulo RFID'
            self.msg_1['fg'] = 'red'
            self.msg_1['text'] = 'Erro de conexão com módulo RFID'
        elif resultado ==  'porta errada':
            resultado = 'Leitura de dados incorreta'
            self.msg_1['fg'] = 'red'
            self.msg_1['text'] = 'FALHA NO TESTE'
        elif resultado:
            self.msg_1['fg'] = 'green'
            self.msg_1['text'] = 'TESTE OK'
        else:
            self.msg_1['fg'] = 'red'
            self.msg_1['text'] = 'Não leu o cartão, tente novamente'

        logging.info(f'Módulo RFID: {resultado}')

    def ativar_botao(self):
        self.botao_3['state'] = tk.NORMAL


class PageNine(tk.Frame): # Sonar

    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)

        self.pady = '20'
        self.width = '12'

        self.primeiroContainer = tk.Frame(self)
        self.primeiroContainer['pady'] = self.pady
        self.primeiroContainer.pack()

        self.segundoContainer = tk.Frame(self)
        self.segundoContainer['pady'] = self.pady
        self.segundoContainer.pack()

        self.terceiroContainer = tk.Frame(self)
        self.terceiroContainer['pady'] = self.pady
        self.terceiroContainer.pack()

        self.quartoContainer = tk.Frame(self)
        self.quartoContainer['pady'] = self.pady
        self.quartoContainer.pack()

        self.quintoContainer = tk.Frame(self)
        self.quintoContainer['pady'] = self.pady
        self.quintoContainer.pack()

        self.sextoContainer = tk.Frame(self)
        self.sextoContainer['pady'] = self.pady
        self.sextoContainer.pack()

        self.setimoContainer = tk.Frame(self)
        self.setimoContainer['pady'] = self.pady
        self.setimoContainer.pack()

        self.oitavoContainer = tk.Frame(self)
        self.oitavoContainer.pack(pady=(175, 20))
        self.oitavoContainer.pack()

        self.nonoContainer = tk.Frame(self)
        self.nonoContainer['pady'] = self.pady
        self.nonoContainer.pack(padx=(45, 45), anchor=tk.E)

        self.titulo_0 = tk.Label(self.primeiroContainer)
        self.titulo_0['text'] = "Teste de Sonar"
        self.titulo_0["font"] = fonteTitulo
        self.titulo_0.pack()

        self.titulo_1 = tk.Message(self.segundoContainer)
        self.titulo_1['text'] = 'Pressione o botão e aproxime algum objeto do sonar para realizar o teste:'
        self.titulo_1['width'] = '420'
        self.titulo_1['justify'] = tk.CENTER
        self.titulo_1["font"] = fonteTexto
        self.titulo_1.pack()

        self.botao_1 = tk.Button(self.terceiroContainer)
        self.botao_1["text"] = "Testar"
        self.botao_1["font"] = fonteTexto
        self.botao_1['command'] = lambda:[self.sonar(), self.ativar_botao()]
        self.botao_1['width'] = self.width
        self.botao_1.pack()

        self.msg_1 = tk.Label(self.quartoContainer)
        self.msg_1['text'] = ''
        self.msg_1["font"] = fonteTeste
        self.msg_1.pack()

        self.titulo_2 = tk.Label(self.quintoContainer)
        self.titulo_2['text'] = ''
        self.titulo_2["font"] = fonteTexto
        self.titulo_2.pack()

        # self.botao_2 = tk.Button(self.sextoContainer)
        # self.botao_2["text"] = "Testar"
        # self.botao_2["font"] = fonteTexto
        # self.botao_2['command'] = None
        # self.botao_2.pack()

        self.msg_2 = tk.Label(self.setimoContainer)
        self.msg_2['text'] = ''
        self.msg_2["font"] = fonteTeste
        self.msg_2.pack()

        self.titulo_3 = tk.Message(self.oitavoContainer)
        self.titulo_3['text'] = ''
        self.titulo_3["font"] = fonteTexto
        self.titulo_3['width'] = '420'
        self.titulo_3['justify'] = tk.CENTER
        self.titulo_3.pack()

        self.botao_3 = tk.Button(self.nonoContainer)
        self.botao_3["text"] = "Próximo"
        self.botao_3["font"] = fonteTexto
        self.botao_3['command'] = lambda: controller.show_frame(PageTen)
        self.botao_3['width'] = self.width
        # self.botao_3['state'] = tk.DISABLED
        self.botao_3['state'] = tk.NORMAL
        self.botao_3.pack(padx=10, side=tk.RIGHT)

        self.botao_4 = tk.Button(self.nonoContainer)
        self.botao_4["text"] = "Voltar"
        self.botao_4["font"] = fonteTexto
        # self.botao_4['command'] = lambda: controller.show_frame(PageEight)
        self.botao_4['command'] = lambda: controller.show_frame(StartPage)
        self.botao_4['width'] = self.width
        self.botao_4['state'] = tk.NORMAL
        self.botao_4.pack(padx=10, side=tk.RIGHT)

    def sonar(self):
        resultado = model.teste_sonar()
        if resultado == False:
            self.msg_1['fg'] = 'red'
            self.msg_1['text'] = 'TESTE FALHOU'
            logging.info(f'Sonar: FALHOU')
        else:
            self.msg_1['fg'] = 'green'
            self.msg_1['text'] = str(resultado)
            logging.info(f'Sonar: OK  - {resultado}')
        logging.info('----------------------------------------')

    def ativar_botao(self):
        self.botao_3['state'] = tk.NORMAL


class PageTen(tk.Frame): # Codigo de barras

    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)

        global my_img
        my_img = ImageTk.PhotoImage(Image.open('/home/pi/caixa-magica-teste-HW/barcode.png'))

        self.pady = '20'
        self.width = '12'

        self.primeiroContainer = tk.Frame(self)
        self.primeiroContainer.pack(pady=(200, 10))

        self.segundoContainer = tk.Frame(self)
        self.segundoContainer.pack(padx=(45, 45), pady=(220, 10))

        self.titulo_0 = tk.Label(self.primeiroContainer, image=my_img)
        self.titulo_0.pack()

        self.botao_1 = tk.Button(self.segundoContainer, text="Sair")
        self.botao_1['font'] = fonteTexto
        self.botao_1['command'] = lambda: [self.destruir(), self.apagar_barcode(), quit]
        self.botao_1['font'] = fonteTexto
        self.botao_1['width'] = self.width
        self.botao_1.pack(padx=10, side=tk.RIGHT)

        self.botao_4 = tk.Button(self.segundoContainer)
        self.botao_4["text"] = "Voltar"
        self.botao_4["font"] = fonteTexto
        # self.botao_4['command'] = lambda: controller.show_frame(PageEight)
        self.botao_4['command'] = lambda: controller.show_frame(StartPage)
        self.botao_4['width'] = self.width
        self.botao_4['state'] = tk.NORMAL
        self.botao_4.pack(padx=10, side=tk.RIGHT)

    def destruir(self):
        app.destroy()
    
    def apagar_barcode(self):
        path = "/home/pi/caixa-magica-teste-HW"
        dir = os.listdir(path)
        for file in dir:
            if file == "barcode.png":
                os.remove(file)

# Gerando número serial antes de entrar nos testes
serial = model.getSerial()
model.generateBarcode(serial)

# Abrindo as telas
app = SeaofBTCapp()
app.mainloop()

# # Criando o arquivo para o teste ser realizado só na primeira vez
# f = open('teste_realizado.txt', 'w')
# f.write('Null')
# f.close()

# # Enviando email com os dados do log
# corpo_email = model.ler_log()
# print(corpo_email)
# model.enviar_email(corpo_email)

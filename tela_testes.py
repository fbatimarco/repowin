from tkinter import *
from tkinter.ttk import *
import tkinter.font as font
import model
import serial
import time
from time import sleep
from datetime import datetime

class telaTestes():
    sizeTela = '600x800'
    root = Tk()
    root1 = Tk()
    frame1 = Frame(root1,height=100,width=100)
    
    datahoraatualstr = datetime.now().strftime('%d/%m/%Y %H:%M:s')

    widthBotao = 14
    heightBotao = 2

    def marcadatahoraacao(self):
        self.datahoraatualstr = datetime.now().strftime('%d/%m/%Y %H:%M:%s')


    def destruir(root):
        #root.destroy()
        print("Erro")

    def teste_internet_modem(self):
        self.frame1.labelResultado['text'] = 'EFETUANDO CONEXAO, AGUARDE...'
        time.sleep(2)

        resultado = model.teste_internet()

        # se a conexao teve exito
        if resultado == True:
            self.frame1.labelResultado['text'] = 'CONEXAO OK'
            #self.frame1.labelResultado['fg'] = '#f0f'
        else:
            self.frame1.labelResultado['text'] = 'CONEXAO FALHOU'
            #self.frame1.labelResultado['fg'] = '#f00'

        self.marcadatahoraacao()
        self.frame1.labelResultado['text'] = self.frame1.labelResultado['text'] + " " + str(self.datahoraatualstr)

    def teste_internet_wifi(self):
        resultado = model.teste_wifi()

        if resultado == True:
            self.frame1.labelResultado['text'] = 'WI-FI PRESENTE'
        else:
            self.frame1.labelResultado['text'] = 'WI-FI FALHOU'
            #self.frame1.labelResultado['fg'] = '#f00'
        
        self.marcadatahoraacao()
        self.frame1.labelResultado['text'] = self.frame1.labelResultado['text'] + " " + str(self.datahoraatualstr)

    # Tela teste internet
    def telaInternet(self):
        #root1 = Tk()
        self.root1.geometry(self.sizeTela)
        self.root1.attributes('-fullscreen', True)

        #frame1 = Frame(self.root1, height=100, width=200)
        self.frame1.titulo_0 = Label(self.frame1, text="Teste Internet")
        self.frame1.titulo_0.pack(pady=12)

        self.frame1.botao1 = Button(self.frame1, text="Testar internet", command=self.teste_internet_modem)
        self.frame1.botao1.pack(pady=12)

        self.frame1.botao2 = Button(self.frame1, text="Testar wi-fi", command=self.teste_internet_wifi)
        self.frame1.botao2.pack(pady=12)

        self.frame1.labelResultado = Label(self.frame1, text="O resultado dos testes sera exibido aqui")
        self.frame1.labelResultado.pack(pady=12)

        self.frame1.botaoSair = Button(self.frame1, text="Sair", command=self.root1.destroy)
        self.frame1.botaoSair['width'] = self.widthBotao
        self.frame1.botaoSair.pack(pady=12)

        self.frame1.pack()

    # tela teste GPS
    def telaGPS(self):
        print("GPS")
    
    def __init__(self):
        
        #self.root = Tk()
        self.root.geometry(self.sizeTela)
        self.root.attributes('-fullscreen', True)
        frame1 = Frame(self.root, height =100, width=200)
        myFont = font.Font(family='Arial')

        # Titulo
        frame1.titulo_0 = Label(frame1, text="TESTES COMPONENTES - CM")
        frame1.titulo_0['justify'] = 'center'
        frame1.titulo_0['font'] = ('Arial', 18, 'bold')
        frame1.titulo_0.pack(pady=12)

        # Botao de teste da Internet
        frame1.botaoInternet = Button(frame1, text="Internet", command=self.telaInternet)
        frame1.botaoInternet['width'] = self.widthBotao
        #rame1.botaoInternet['font'] = myFont
        #frame1.botaoInternet['height'] = heightBotao
        frame1.botaoInternet.pack(pady=12)

        # teclando "1", faz mesmo comportamento do botao
        #self.root.bind('1',self.telaInternet())

        # Botao para teste GPS
        frame1.botaoGPS = Button(frame1, text="GPS", command=self.telaGPS)
        frame1.botaoGPS['width'] = self.widthBotao
        frame1.botaoGPS.pack(pady=12)


        # Botao Sair
        frame1.botaoSair = Button(frame1, text="Sair", command=quit)
        frame1.botaoSair['width'] = self.widthBotao
        frame1.botaoSair.pack(pady=12)

        # teclando "Q", faz mesmo comportamento do botao
        self.root.bind('q', quit)

        frame1.pack()

        mainloop()

app = telaTestes()

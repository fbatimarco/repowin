from imutils.video import VideoStream
from pyzbar import pyzbar
import imutils
import cv2
import time
import RPi.GPIO as GPIO
import serial
import threading
import requests
import smtplib, ssl 
import pdb
from wifi import Cell, Scheme #Teste de wifi
import socket
import uuid

from barcode.writer import ImageWriter
from barcode import generate
import barcode


import json
CONFIG = None
with open('/home/pi/caixa-magica/core/config.json') as json_data:
    CONFIG = json.load(json_data)

class serial_GPS(serial.Serial):
    def get_loc(self):    
        if not self.is_open:self.open()
        info_GPS = self.read_until(b'VTG') 
        start = info_GPS.find(b"RMC")
        final = info_GPS.find(b"W",start)
        GPRMC = info_GPS[start + 1: final + 1]
        self.close()
        if final <= 0: return None 
        return GPRMC

    def is_working(self):
        if not self.read_until(b'VTG'):
            return False
        else:
            return True

class Catraca(object):
    def __init__(self, Catraca, RetornoCatraca):
        self.gpio_ctc = Catraca
        self.gpio_rc = RetornoCatraca

    def liberar(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.gpio_ctc,GPIO.OUT)
        GPIO.output(self.gpio_ctc,GPIO.HIGH)

    def travar(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.gpio_ctc,GPIO.OUT)
        GPIO.output(self.gpio_ctc,GPIO.LOW)

    # Volta True se catraca aberta, False se catraca travada
    def is_open(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.gpio_rc,GPIO.IN,pull_up_down = GPIO.PUD_UP)
        # leitura de RC:
        # 1 - catraca travada
        # 0 - catraca liberada
        return not bool(GPIO.input(self.gpio_rc))

class Buzzer(object):
    def __init__(self, pin):
        self.pino = pin

    def test(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pino,GPIO.OUT)
        for x in range(3):
            GPIO.output(self.pino,GPIO.HIGH)
            time.sleep(0.1)
            GPIO.output(self.pino,GPIO.LOW)
            time.sleep(0.1)
            GPIO.output(self.pino,GPIO.HIGH)
            time.sleep(0.1)
            GPIO.output(self.pino,GPIO.LOW)
            time.sleep(0.1)
            GPIO.output(self.pino,GPIO.HIGH)
            time.sleep(0.1)
            GPIO.output(self.pino,GPIO.LOW)
            time.sleep(0.5)

class MiFareRead():
    port = ''
    def __init__(self, port= CONFIG['pins']['mifare']):
    # def __init__(self, port= "/dev/ttyS0"):
    # def __init__(self, port= "/dev/ttyAMA0"):
        self.port = port
    
    def int_to_bytes(self, val):
        return val.to_bytes(16, byteorder='big')
    
    def bytes_to_int(self, byte):
        return int.from_bytes(byte, byteorder='big')

    def get_card_number(self):
        with serial.Serial(self.port, 19200) as ser:
            print("lendo")
            while True:
                print(ser.read())
            r = ser.read(9)
            s = str(r.hex())
            ser.close()
            return s[6:14]
        return False

    def select_card(self):
        while True:
            r = self.send([0x01, 0x01, 0x73])
            if r == 'serial': return r
            if r == 'porta errada': return r
            if r != False:
                s = ''.join(r)
                print(s)
                if r[3] != '4e' and r[4] != '4f':
                    print(s[6:14])
                    return s[6:14]
                else:
                    print("Nenhum cartao")
                    return False
            else:
                return False
    
    def generate_xor(self, value):
        cs = 0
        for byte in value:
            cs = cs ^ byte
        arr = bytes([0x02] + value + [cs, 0x03])
        return arr
    
    def send(self, command):
        command = self.generate_xor(command)
        print("sending", command)
        try:
            with serial.Serial(self.port, 19200, timeout=1) as ser:
                ser.write_timeout = 3
                r = ser.write(command)
                end = False
                response = []
                #Timeout de 3 segundos
                Timeout = time.time() + 3 
                while not end:
                    r = ser.read()
                    response.append(r.hex())
                    if r.hex() == "03" or r == b'':
                        end = True
                    if time.time() > Timeout:
                        print('timeout de leitura')
                        break
                if len(response) >= 20: return 'porta errada'
                print(response)
                ser.close()
                return response
        except:
            print('Erro de conexão com módulo RFID')
            return 'serial'
    
    def read(self, block):
        c = [0x01, 0x02, 0x72, block]
        return self.send(c)
    
    def write(self, block, data):
        c = [0x01, 0x12, 0x77, block] + data
        return self.send(c)

    def login(self, sector=0x00, key=[0x00,0x00,0x00,0x00,0x00,0x00]):
        c = [0x01,0x09,0x6c,sector,0xAA] + key 
        command = self.generate_xor(c)
        with serial.Serial(self.port, 19200) as ser:
            r = ser.write(command)
            response = []
            while True:
                r = ser.read()
                response.append(r.hex())
                if r.hex() == "03" or r == b'':
                    break
            print(response)
            ser.close()
            if (response[3] == '4c' and response[4] == '4d'):
               return True
            return False
        return False

## Teste de leitura 
def teste_rfid():
    mifare = MiFareRead()
    time.sleep(0.1)
    result = mifare.select_card()
    print('Resultado'+str(result))
    return result

# Testa comunicação da Rasp com o GPS via serial
#  - retorno = Boolean (True or False) 
def teste_GPS_comm():
    try:
        entrada_gps = CONFIG['pins']['gps']
        GPS = serial_GPS(entrada_gps)
        print(GPS.port)
        GPS.timeout = 5
        print('Timeout: '+str(GPS.timeout)+' seg')
        # Comunicacao com GPS
        print('Comunicacao serial com GPS: ',GPS.is_working())
        return GPS.is_working()
    except serial.SerialException as e:
        print(e)
        print('xxxx Erro na criacao de porta xxxxx ')
        return False
    except:
        print("xxx Erro desconhecido xxx")
        return False



# Lê serial do GPS e obtém localização
#  - retorno se tiver sinal = string com GPRMC 
#  - retorno sem sinal      = None
def teste_GPS_loc():
    try:
        entrada_gps = CONFIG['pins']['gps']
        GPS = serial_GPS(entrada_gps)
        print(GPS.port)
        GPS.timeout = 5
        print('Timeout: '+str(GPS.timeout)+' seg')
        # Comunicacao com GPS
        print('Comunicacao serial com GPS: ', GPS.is_working())
        if GPS.is_working():
            localizacao = GPS.get_loc()
            print('Resultado:' + str(localizacao))
            return localizacao
        return None
    except:
        print("xxx Erro desconhecido xxx")
        return None

# Aciona o Buzzer.
# - sem retorno
def teste_buzzer():
    try:
        buzzer_pin = CONFIG['pins']['buzzer'] #16
        buzzer = Buzzer(buzzer_pin)
        buzzer.test()
    except:
        print("xxx Erro xxx")
    return    

# Aciona a câmera. Aguarda a leitura de QR Code de teste(msg:'')
# timeout Z, em segundos. Se Z=0, não tem timeout. Sw dependerá de ação do usuário para sair do teste, caso a câmera não funcione 
# - retorno se conseguiu ler    = Boolean (True)
# - retorno se atingiu timeout  = Boolean (False)
def teste_camera(Z):
    try:
        # initialize the video stream and allow the camera sensor to warm up
        print("[INFO] starting video stream...")
        vs = VideoStream(usePiCamera=True).start()
        time.sleep(2.0)
        found = set()
        print("[INFO] PASSSOu")

        # loop over the frames from the video stream
        while True:
            # grab the frame from the threaded video stream and resize it to
            # have a maximum width of 400 pixels
            frame = vs.read()
            frame = imutils.resize(frame, width=400)
            # find the barcodes in the frame and decode each of the barcodes
            barcodes = pyzbar.decode(frame)
            # loop over the detected barcodes
            for barcode in barcodes:
                # extract the bounding box location of the barcode and draw
                # the bounding box surrounding the barcode on the image
                (x, y, w, h) = barcode.rect
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                # the barcode data is a bytes object so if we want to draw it
                # on our output image we need to convert it to a string first
                barcodeData = barcode.data.decode("utf-8")
                barcodeType = barcode.type

                # if barcodeData == ('TESTE_CAMERA_PARA_DIREITA'):
                if barcodeData == ('gAAAAABfh2I9a9IQCZVlARuge_fr01PRPlEhzo8ntr9gCexEKJGP6z54S-ZEZMCbt4c8cpIc429tD_QavJxrKX_5naGZI3pp42dGuz9nhcpl2tH1RlUCtVI='):
                    print("[INFO] Achoooo")
                    cv2.destroyAllWindows()
                    vs.stop()
                    return True
                # draw the barcode data and barcode type on the image
                text = "{} ({})".format(barcodeData, barcodeType)
                cv2.putText(frame, text, (x, y - 10),
                	cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)      

            # show the output frame
            cv2.imshow("Barcode Scanner", frame)
            key = cv2.waitKey(1) & 0xFF

            # if the `q` key was pressed, break from the loop
            if key == ord("q"):
            	break

        # close the output CSV file do a bit of cleanup
        print("[INFO] cleaning up...")
        cv2.destroyAllWindows()
        vs.stop()
    except:
        print("Falha na Camera")
        return False

#### Testes de Botão
# Espera botão ser pressionado 3 vezes
botao_testado = threading.Event()

def button_pressed():
    botao_testado.clear()
    print('debug2.A')
    for x in range(3):
        print(GPIO.wait_for_edge(pino_botoeira,GPIO.FALLING))
        # input()
        time.sleep(0.2)
    print('debug2.B')
    botao_testado.set()

# Testa Botão BG (Benefício Gratuidade)
# Aguarda o usuário apertar botão
#  - retorno = Boolean (True)
def teste_bot_BG():
    global pino_botoeira
    pino_botoeira = CONFIG['pins']['botao_G'] #20
    BG_pin = pino_botoeira
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(BG_pin,GPIO.IN,pull_up_down = GPIO.PUD_UP)
    # Cehcar se ele está em nível Alto quando não pressionado
    if GPIO.input(BG_pin) == 1:
        thread = threading.Thread(target=button_pressed)
        print('debug.1A')
        thread.start()
        if botao_testado.wait(timeout = 5):
            print('Teste ocorreu dentro do timeout')
            GPIO.cleanup()
            return True
        print('Extrapolou timeout')
        GPIO.cleanup()
        return False
    else:
        return False
    
# Testa Botão BP (Benefício P)
# Aguarda o usuário apertar botão
#  - retorno = Boolean (True) 
def teste_bot_BP():
    global pino_botoeira
    pino_botoeira = CONFIG['pins']['botao_P'] #21
    BP_pin = pino_botoeira
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(BP_pin,GPIO.IN,pull_up_down = GPIO.PUD_UP)
    # Cehcar se ele está em nível Alto quando não pressionado
    if GPIO.input(BP_pin) == 1:
        thread = threading.Thread(target=button_pressed)
        print('debug.1A')
        thread.start()
        if botao_testado.wait(timeout = 5):
            print('Teste ocorreu dentro do timeout')
            GPIO.cleanup()
            return True
        print('Extrapolou timeout')
        GPIO.cleanup()
        return False
    else:
        return False


# Testa Acionamento da catraca e pino de Retorno da Catraca(RC)
#  - retorno = Boolean (True or False)
def teste_catraca():
    GPIO.setwarnings(False) # Ignore warning for now
    GPIO.setmode(GPIO.BCM) # Use physical pin numbering
    RC_pin = CONFIG['pins']['retorno_catraca'] # 26
    CTC_pin = CONFIG['pins']['catraca'] #12
    GPIO.setup(RC_pin,GPIO.IN,pull_up_down=GPIO.PUD_UP)
    GPIO.setup(CTC_pin,GPIO.OUT)
    
    GPIO.output(CTC_pin, GPIO.HIGH)
    ret = 1
    time.sleep(0.01)
    Timeout = time.time() + 20
    while (ret == 1):
        ret = GPIO.input(RC_pin)
        if time.time() > Timeout:
            GPIO.output(CTC_pin, GPIO.LOW)
            print('timeout de catraca')
            return False
    GPIO.output(CTC_pin, GPIO.LOW)
    return True

    # RC_pin = CONFIG['pins']['retorno_catraca'] # 26
    # CTC_pin = CONFIG['pins']['catraca'] #12
    # catraca = Catraca(CTC_pin, RC_pin)
    # catraca.liberar()
    # time.sleep(0.5)
    # print('Catraca liberada? ' + str(catraca.is_open()))
    # if not catraca.is_open(): return False
    # catraca.travar()
    # time.sleep(0.5)
    # print('Catraca liberada? ' + str(catraca.is_open()))
    # if catraca.is_open(): return False
    # return True

# Testa de conexão da internet
#  - retorno = Boolean (True or False)
def teste_internet():
    url = "http://www.google.com"
    timeout = 5
    try:
        request = requests.get(url, timeout=timeout)
        print("Conectado na Internet")
        return True
    except (requests.ConnectionError, requests.Timeout) as exception:
        print("Sem conexão com a internet")
        return False

# Testa wifi
#  - retorno = Boolean (True or False)
def teste_wifi():
    cells = Cell.all('wlan0')
    lista = list(map(str,cells))
    if lista: return True
    else: return False

# Testa Sonar
#  - retorno =  distance(float) se estiver OK
#               FALSE se NÂO OK
def teste_sonar():
    #GPIO Mode (BOARD / BCM)
    GPIO.setmode(GPIO.BCM)
 
    #set GPIO Pins
    GPIO_TRIGGER = 6# 4
    GPIO_ECHO = 19 # 18
 
    #set GPIO direction (IN / OUT)
    GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
    GPIO.setup(GPIO_ECHO, GPIO.IN)

    # set Trigger to HIGH
    GPIO.output(GPIO_TRIGGER, True)
 
    # set Trigger after 0.01ms to LOW
    time.sleep(0.1)
    GPIO.output(GPIO_TRIGGER, False)
 
    StartTime = time.time()
    StopTime = time.time()
    Timeout = time.time() + 3

    # save StartTime
    while GPIO.input(GPIO_ECHO) == 0:
        #print("start", GPIO.input(GPIO_ECHO))
        StartTime = time.time()
        if time.time() > Timeout:
            print('timeout ECHO: 0')
            return False
 
    # save time of arrival
    while GPIO.input(GPIO_ECHO) == 1:
        #print("arrival", GPIO.input(GPIO_ECHO))
        StopTime = time.time()
        if time.time() > Timeout:
            print('timeout ECHO: 1')
            return False

 
    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2
 
    return distance


# Enviar dados por email da CM, para equipe da CM
def enviar_email(msg_log):
    port = 465  # Para padrão SSL
    smtp_server = "smtp.gmail.com"

    remetente = "cm.teste.hw@gmail.com"
    password = 'b2ml2020HW'
    # destinatario = ['mateus.prado@b2ml.com.br']
    destinatario = ['mateus.prado@b2ml.com.br','guilherme.luz@b2ml.com.br','filipe.soares@b2ml.com.br ','otavio@b2ml.com.br','pedro.bonafe@b2ml.com.br']
    LOG = 'Exemplo de texto: LOG'
    mensagem = """\
Subject: Relatorio de Testes da CM Serial - """ + getSerial() + """

""" + msg_log
    # Create a secure SSL context
    context = ssl.create_default_context()
    with smtplib.SMTP_SSL("smtp.gmail.com", port, context=context) as server: 
        server.login(remetente, password)
        server.sendmail(remetente, destinatario, mensagem)

# Faz a leitura do LOG e retorna uma variável string
def ler_log():
    file = open('/home/pi/caixa-magica/teste_HW/funcionamento_hardwares.log', 'r')
    lines = file.read().splitlines()
    file.close()
    email_str = ''
    for x in lines:
        email_str += str(x) + '\n'
    # print(email_str)
    return email_str

## Funções para gerar o código de barras
# Identificar número serial da caixa
def getSerial():
    cpuserial = ''
    try:
        with open ('/proc/cpuinfo', 'r') as f:
        # f = open('/proc/cpuinfo', 'r')
            for line in f:
                if line[0:6] == 'Serial':
                    cpuserial = line[10:26]
            f.close()
    except:
        cpuserial = 'Error'
    return cpuserial

# Gerar o codigo de barras
def generateBarcode(serial):
    try:
        generate('CODE128', serial, writer=ImageWriter(), output='/home/pi/caixa-magica/teste_HW/barcode')
    except:
        pass


# teste_bot_BG()

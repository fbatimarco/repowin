#https://note.nkmk.me/en/python-pillow-qrcode/

## pip install qrcode
## pip install pillow

import qrcode

img = qrcode.make('TESTE_CAMERA_PARA_DIREITA')

print(type(img))
print(img.size)
# <class 'qrcode.image.pil.PilImage'>
# (290, 290)

img.save('qrcode_BS.png')
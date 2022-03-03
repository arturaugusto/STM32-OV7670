from PIL import Image
import serial
import time
import requests
import telegram_secrets
import io

"""
* Creating the rfcomm serial port linked to bluetooth device * 

First check if the bluetooth from your computer is not blocled
> sudo rfkill

If it is blocked, unblock using:
> sudo rfkill unblock 1

Replace `1` with the correct index then
check the status of you bluetooth:
> sudo hciconfig hci0

Manage your bluetooth connections:
> sudo bluetoothctl

From the new prompt, scan for devices
>[bluetooth]# scan on

Pair using:

>[bluetooth]# pair 00:00:00:00:00:00

Replace the zeros with correct address
Than type "exit" to quit the bluetooth ctl prompt

Now create the `/dev/rfcomm0` from shell using the command:

> sudo rfcomm bind hci0 00:00:00:00:00:00

Now should exist a /dev/rfcomm0
ref: https://falb18.github.io/tinkering_at_night/bluetooth-hc05-linux.html
"""

def grab_image():
  """
  get data from bluetooth<>serial
  return PIL image
  """
  with serial.Serial('/dev/rfcomm0', 19200, timeout=20) as ser:
    time.sleep(0.2)
    ser.flush()
    #print(ser.name)
    time.sleep(0.2)
    ser.write(b'x')
    time.sleep(0.2)
    data = ser.read(8192)

  #img = Image.new('L', (128, 64))
  #img.putdata(data)
  #img = img.transpose(Image.FLIP_TOP_BOTTOM)
  #img.save('../test.bmp')
  return img
  #curl -s -X POST https://api.telegram.org/bot{}/sendMessage -d chat_id={} -d text="blah"
  
def post_msg():
  img = Image.open('../test.bmp')
  buf = io.BytesIO()
  buf.name = 'img.jpeg'
  img.save(buf, format='JPEG')
  buf.seek(0)
  files = {'photo': buf}

  r = requests.post(f"https://api.telegram.org/bot{telegram_secrets.token}/sendPhoto", # sendMessage to text only
    files = files,
    data = {
    "chat_id": telegram_secrets.chat_id,
    "text": "asdasd"})
  print(r.status_code)

if __name__ == '__main__':
  post_msg()
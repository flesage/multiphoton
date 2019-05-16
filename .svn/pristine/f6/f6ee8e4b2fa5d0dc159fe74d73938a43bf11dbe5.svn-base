'''
Created on Nov 27, 2015

@author: flesage
'''
import time
from base import liomio
from base import imgenerator

if __name__ == '__main__':

    saver=liomio.DataSaver('/Users/flesage/Desktop/test.hd5')
    saver.setDatasetName('MyFirstDataset')
    saver.setBlockSize(512)
    saver.addAttribute('nx', 512)
    saver.addAttribute('ny', 512)
    conn = saver.startSaving()

    # Generate images and push in queue with no loss
    imgen=imgenerator.RandomImage16Bits(100,512,512)
    imgen.setDataConsumer(saver, True, 0)
    imgen.start()
    for i in range(100):
        print i
        time.sleep(1)
        if conn.poll():
            print conn.recv()

    imgen.stop()

    saver.stopSaving()
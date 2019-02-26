'''
Created on 2015-12-09

@author: LiomW17
'''
from base.Motors import RotationMotor

if __name__ == '__main__':
    power = RotationMotor('COM4')
    print power.getPos()
    power.goTo(45)
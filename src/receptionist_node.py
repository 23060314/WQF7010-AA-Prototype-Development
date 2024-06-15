#!/usr/bin/env python

import os
import rospy
from sound_play.libsoundplay import SoundClient
from opencv_apps.msg import FaceArrayStamped

class ReceptionistNode:
  def __init__(self):
    # initializing node, subscriber and sound client object
    rospy.init_node('receptionist_node')
    # register a callback function upon the node is shutting down
    #rospy.on_shutdown(self.cleanup)
    
    self.subscriber = rospy.Subscriber('/face_detection/faces', FaceArrayStamped, self.greeting)
    self.soundhandler = SoundClient(blocking=True)
    self.face_detected = False

    # pause execution of the node for 1 second to let the sound client to connect to the sound_play server
    rospy.sleep(1)

    # stop if any sound_play processes are in the queue
    self.soundhandler.stopAll()

    os.system('clear')
    print('Look at the camera for a second to trigger FSKTM Receptionist...')

  def cleanup(self):
    self.soundhandler.stopAll()
    rospy.signal_shutdown('Shutdown requested by user')

  def greeting(self, msg):
    if not self.face_detected and self.detect_face(msg.faces):
      self.face_detected = True
      self.subscriber.unregister()

      self.soundhandler.say('Welcome to Faculty of Computer Science and Information Tectnology, University Ma-la-ya.')
      self.soundhandler.say('Please select an option below by entering the number.')

      os.system('clear')
      print('============================================')
      print('=             Welcome to FSKTM             =')
      print('============================================')
      print('=                                          =')
      print('= (1) I am a visitor.                      =')
      print('=                                          =')
      print('= (2) I have an appointment with lecturer. =')
      print('=                                          =')
      print('= (3) I have a parcel to deliver.          =')
      print('=                                          =')
      print('============================================')
      print('')
      selection = input('Enter an option (0 to exit): ')

      if selection == 1:
        self.soundhandler.say('Feel free to hang around. Have a good day!')
      elif selection == 2 or selection == 3: 
        print('')
        print('')
        print('')
        print('==================================================')
        print('=                 Lecturer Name                  =')
        print('==================================================')
        print('=                                                =')
        print('= (1) Dr. Erma Rahayu Binti Mohd Faizal Abdullah =')
        print('=                                                =')
        print('= (2) Dr. Nurul Binti Japar                      =')
        print('=                                                =')
        print('= (3) Dr. Saw Shier Nee                          =')
        print('=                                                =')
        print('= (4) Dr. Zati Hakim Binti Azizul Hasan          =')
        print('=                                                =')
        print('= (5) Dr. Norisma Binti Idris                    =')
        print('=                                                =')
        print('= (6) Dr. Muhammad Shahreeza Safiruz Bin Kassim  =')
        print('=                                                =')
        print('= (7) Dr. Aznul Qalid Bin Md Sabri               =')
        print('=                                                =')
        print('= (8) Dr. Norjihan Binti Abdul Ghani             =')
        print('=                                                =')
        print('= (9) Dr. Unaizah Hanum Binti Obaidellah         =')
        print('=                                                =')
        print('==================================================')
        print('') 
        lecturer_selection = input('Select the lecturer (0 to exit): ')
        
        lecturer_name = self.get_lecturer_name(lecturer_selection)
        if lecturer_name is not None:
          print('')
          sentence = lecturer_name + " room at management office, level 3 of Faculty of Computer Science and Information Tectnology."
          print(sentence)
          self.soundhandler.say(sentence)

      # pause execution of the node for 3 seconds before re-subscribe to face detection topic
      rospy.sleep(3)

      self.subscriber = rospy.Subscriber('/face_detection/faces', FaceArrayStamped, self.greeting)
      self.face_detected = False

      os.system('clear')
      print('Look at the camera for a second to trigger FSKTM Receptionist...')

  def detect_face(self, faces):
    if faces:
      for f in faces:
        # uncomment the following lines to log detected faces and eyes data.
        #rospy.loginfo('Face data: %s', f.face)
        #rospy.loginfo('Eye data: %s', f.eyes)

        if f.face and f.eyes:
          return True
    return False

  def get_lecturer_name(self, selection):
    if selection == 1:
      return 'Dr. Erma'
    elif selection == 2:
      return 'Dr. Nurul'
    elif selection == 3:
      return 'Dr. Saw'
    elif selection == 4:
      return 'Dr. Zati'
    elif selection == 5:
      return 'Dr. Norisma'
    elif selection == 6:
      return 'Dr. Shahreeza'
    elif selection == 7:
      return 'Dr. Aznul'
    elif selection == 8:
      return 'Dr. Norjihan'
    elif selection == 9:
      return 'Dr. Unaizah'
    else:
      return None

if __name__ == '__main__':
  ReceptionistNode()

  while not rospy.is_shutdown():
    try:      
      rospy.spin()
    except rospy.ROSInterruptException:
      pass


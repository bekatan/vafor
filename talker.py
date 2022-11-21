#!/usr/bin/env python

import rospy
import spacy
from std_msgs.msg import String
import speech_recognition as sr


# SENTENCES = ['can you bring me the cup',
#              'can you pass me the book',
#              'I need that mouse',
#              'I want to eat a carrot',
#              'bring me an orange',
#              'bring that apple to me']



def getTarget():
    nlp = spacy.load('en_core_web_sm')
    sentence = ""

    try:
        with sr.Microphone() as source2:
            r.adjust_for_ambient_noise(source2, duration=0.5)
            
            audio2 = r.listen(source2)
            
            sentence = r.recognize_google(audio2)
            sentence = sentence.lower()

            print("Did you say ",sentence)
            
    except sr.RequestError as e:
        print("Could not request results; {0}".format(e))
        
    except sr.UnknownValueError:
        print("unknown error occurred")
        
    if not len(sentence):
        return ""

    doc = nlp(sentence)

    target = ""

    for token in doc:
        if token.dep_ == 'dobj':
            target = token.text
            
    return target

def talker():
    pub = rospy.Publisher('target', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)
    target = ''
    while not rospy.is_shutdown():
        temp = getTarget()
        if temp != '':
            target = temp
        pub.publish(target)
        rate.sleep()

if __name__ == '__main__':
    r = sr.Recognizer()
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

            
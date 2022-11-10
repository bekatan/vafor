import rospy
import spacy
from std_msgs.msg import String

SENTENCES = ['can you bring me the cup',
             'can you pass me the book',
             'I need that mouse',
             'I want to eat a carrot',
             'bring me an orange',
             'bring that apple to me']

def getTarget(sentence):
    nlp = spacy.load('en_core_web_sm')

    doc = nlp(sentence)

    target = ""

    for token in doc:
        if token.dep_ == 'dobj':
            target = token.text
            
    return target

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        target = getTarget(SENTENCES[0])
        # rospy.loginfo(target)
        pub.publish(target)
        rate.sleep()
# for sentence in SENTENCES:
#     print(getTarget(sentence))

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
import rospy
from teleop_youbot_robot import MoveBase

def main():
    teste = MoveBase()
    teste.baseGoToGoal(0.1,0.2)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('teleop_youbot_robot')
    main()
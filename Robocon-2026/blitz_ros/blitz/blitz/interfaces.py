# from robot_interfaces.msg import Counter
from blitz import Blitz
from my_custom_interfaces.msg import BnoReading
from my_custom_interfaces.msg import Lidar 
from my_custom_interfaces.msg import Pwm

blitz_interfaces = {str : Blitz}

# add an interface in this form
blitz_interfaces = {

    # "counter": Blitz(

    #     # the topic whose data you want to send to the mcu
    #     # this is the topic you will PUBLISH to
    #     topic="/counter",

    #     # msg id should match the id in the microcontroller
    #     msg_id=1,

    #     # format of the msg packet 
    #     # uint8 : B, int8 b, uint16 H, int16 h, uint32 I, int32 i, float32 f
    #     struct_fmt="hhff",

    #     # fields just as in the ros interface 
    #     fields=["a","b","c","d"],

    #     # ros interface name, import it first
    #     ros_msg=Counter,

    #     # false id you are writing to the microcontroller
    #     from_mcu=False
    # ),

    # "counter_response": Blitz(

    #     # to receive from the microcontroller

    #     # the data coming from the mcu will be published in this topic
    #     # this is the topic you will SUBSCRIBE drom
    #     topic="/counter_response",

    #     # msg_id should match the id in the microcontroller
    #     msg_id=2,

    #     # same as above
    #     struct_fmt="hhff",
    #     fields=["a","b","c","d"],
    #     ros_msg=Counter,

    #     # set this as true rest same
    #     from_mcu=True
    # ),
    "bno" : Blitz(
    topic="/current_angle",
    msg_id=1,
    struct_fmt="f",
    fields=["current_angle"],
    ros_msg=BnoReading,
    from_mcu=True
    ),
    "lidar":Blitz(
    topic="/distance",
    msg_id=2,
    struct_fmt="f",
    fields=["distance"],
    ros_msg=Lidar,
    from_mcu=True
    ),
    "Pwm":Blitz(
    topic="/velocity",
    msg_id=3,
    struct_fmt="ffff",
    fields=["m1","m2","m3","m4"],
    ros_msg=Pwm,
    from_mcu=False
    )
    
    

}

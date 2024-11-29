robot_dict = {
    0:'origin',
    1:'mingna',
    2:'vivian',
    3:'gina',
    4:'lucas',
    5:'daisy',
    6:'henry',
    7:'dorie',
    8:'luna',
    9:'robert',
    10:'fiona',
    11:'bingda',
    12:'jetson1'
}


def find_id_by_robot_name(robot_name):
    return next((key for key, value in robot_dict.items() if value == robot_name), None)
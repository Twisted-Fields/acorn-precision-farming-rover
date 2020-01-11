import click


UP_KEYCODE = '\x1b[A'
LEFT_KEYCODE = '\x1b[D'
RIGHT_KEYCODE = '\x1b[C'
DOWN_KEYCODE = '\x1b[B'



class Drive():
    def __init__(self, name):
      self.name = name
      self.home_position = 0


front_left = Drive("front_left")
front_right = Drive("front_right")
rear_right = Drive("rear_right")
rear_left = Drive("rear_left")


odrives = [front_left, front_right, rear_right, rear_left]

def adjust_steering(odrives):
    click.echo('Adjust Steering? [y/n] ', nl=False)
    c = click.getchar()
    click.echo()
    if c != 'y':
        return
    index = 0
    while True:
        drive = odrives[index]
        print("Adjusting Odrive: {} with home position: {}".format(drive.name, drive.home_position))
        c = click.getchar()
        click.echo()
        if c == 'y':
            click.echo('We will go on')
        elif c == 'd':
            click.echo('Done adjusting steering.')
            return
        elif c == UP_KEYCODE:
            print("UP")
            index += 1
            if index >= len(odrives):
                index = 0
        elif c == LEFT_KEYCODE:
            drive.home_position -= 10
            print("LEFT")
        elif c == RIGHT_KEYCODE:
            drive.home_position += 10
            print("RIGHT")
        elif c == DOWN_KEYCODE:
            print("DOWN")
            index -= 1
            if index < 0:
               index = len(odrives) - 1
        else:
            print(repr(c))

adjust_steering(odrives)

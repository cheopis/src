import subprocess
def get_screen_resolution():
    output = subprocess.Popen('xrandr | grep "\*" | cut -d" " -f4',shell=True, stdout=subprocess.PIPE).communicate()[0]
    resolution = output.split()[0].split(b'x')
    screensize = [int(resolution[1]), int(resolution[0])]
    return screensize

A = [1,1]
opa = map(lambda x,y: int(x)/y,tuple(get_screen_resolution()),A)
print(opa)
#!/usr/bin/env python

import random, os.path

#import basic pygame modules
import pygame
from pygame.locals import *

#see if we can load more than standard BMP
if not pygame.image.get_extended():
    raise SystemExit("Sorry, extended image module required")


#game constants
MAX_SHOTS      = 2      #most player bullets onscreen
ALIEN_ODDS     = 22     #chances a new alien appears
BOMB_ODDS      = 60    #chances a new bomb will drop
ALIEN_RELOAD   = 12     #frames between new aliens
SCREENRECT     = Rect(0, 0, 640, 480)
SCORE          = 0

main_dir = os.path.split(os.path.abspath(__file__))[0]

def load_image(file):
    "loads an image, prepares it for play"
    file = os.path.join(main_dir, 'data', file)
    try:
        surface = pygame.image.load(file)
    except pygame.error:
        raise SystemExit('Could not load image "%s" %s'%(file, pygame.get_error()))
    return surface.convert()

def load_images(*files):
    imgs = []
    for file in files:
        imgs.append(load_image(file))
    return imgs


class dummysound:
    def play(self): pass

def load_sound(file):
    if not pygame.mixer: return dummysound()
    file = os.path.join(main_dir, 'data', file)
    try:
        sound = pygame.mixer.Sound(file)
        return sound
    except pygame.error:
        print ('Warning, unable to load, %s' % file)
    return dummysound()

# This class handles sprite sheets
# This was taken from www.scriptefun.com/transcript-2-using
# sprite-sheets-and-drawing-the-background
# I've added some code to fail if the file wasn't found..
# Note: When calling images_at the rect is the format:
# (x, y, x + offset, y + offset)
 
class spritesheet(object):
    def __init__(self, filename):
        try:
            self.sheet = pygame.image.load(filename).convert()
        except pygame.error, message:
            print 'Unable to load spritesheet image:', filename
            raise SystemExit, message
    # Load a specific image from a specific rectangle
    def image_at(self, rectangle, colorkey = None):
        "Loads image from x,y,x+offset,y+offset"
        rect = pygame.Rect(rectangle)
        image = pygame.Surface(rect.size).convert()
        image.blit(self.sheet, (0, 0), rect)
        if colorkey is not None:
            if colorkey is -1:
                colorkey = image.get_at((0,0))
            image.set_colorkey(colorkey, pygame.RLEACCEL)
        return image
    # Load a whole bunch of images and return them as a list
    def images_at(self, rects, colorkey = None):
        "Loads multiple images, supply a list of coordinates" 
        return [self.image_at(rect, colorkey) for rect in rects]
    # Load a whole strip of images
    def load_strip(self, rect, image_count, colorkey = None):
        "Loads a strip of images and returns them as a list"
        tups = [(rect[0]+rect[2]*x, rect[1], rect[2], rect[3])
                for x in range(image_count)]
        return self.images_at(tups, colorkey)
    
class SpriteStripAnim(object):
    """sprite strip animator
    
    This class provides an iterator (iter() and next() methods), and a
    __add__() method for joining strips which comes in handy when a
    strip wraps to the next row.
    """
    def __init__(self, filename, rect, count, colorkey=None, loop=False, frames=1):
        """construct a SpriteStripAnim
        
        filename, rect, count, and colorkey are the same arguments used
        by spritesheet.load_strip.
        
        loop is a boolean that, when True, causes the next() method to
        loop. If False, the terminal case raises StopIteration.
        
        frames is the number of ticks to return the same image before
        the iterator advances to the next image.
        """
        colorkey=Color(96,128,192)
        #colorkey = image.get_at((0,0))
        colorkey = -1
        self.filename = filename
        #ss = spritesheet.spritesheet(filename)
        ss = spritesheet(filename)
        self.images = ss.load_strip(rect, count, colorkey)
        self.i = 0
        self.loop = loop
        self.frames = frames
        self.f = frames
    def iter(self):
        self.i = 0
        self.f = self.frames
        return self
    def next(self):
        if self.i >= len(self.images):
            if not self.loop:
                raise StopIteration
            else:
                self.i = 0
        image = self.images[self.i]
        self.f -= 1
        if self.f == 0:
            self.i += 1
            self.f = self.frames
        return image
    def __add__(self, ss):
        self.images.extend(ss.images)
        return self

def SpriteSheetExample2():
    import sys
    from pygame.locals import Color, KEYUP, K_ESCAPE, K_RETURN
     
    surface = pygame.display.set_mode((350,400))
    FPS = 12
    frames = FPS / 12
#    strips = [
#        SpriteStripAnim('Explode1.bmp', (0,0,24,24), 8, 1, True, frames),
#        SpriteStripAnim('Explode2.bmp', (0,0,12,12), 7, 1, True, frames),
#        SpriteStripAnim('Explode3.bmp', (0,0,48,48), 4, 1, True, frames) +
#        SpriteStripAnim('Explode3.bmp', (48,48,48,48), 4, 1, True, frames),
#        SpriteStripAnim('Explode4.bmp', (0,0,24,24), 6, 1, True, frames),
#        SpriteStripAnim('Explode5.bmp', (0,0,48,48), 4, 1, True, frames) +
#        SpriteStripAnim('Explode5.bmp', (48,48,48,48), 4, 1, True, frames),
#    ]
   # __init__(self, filename, rect, count, colorkey=None, loop=False, frames=1)
   # rect: x,y,x+offset,y+offset
    topLeft = [113+1,72+1]
    botRight = [145,96]
    botLeft = [113,96]
    topRight = [143,72]
    width,height = botRight[0] - topLeft[0],botRight[1] - topLeft[1]
    x5 = width
    y5 = height
    
    sizes = ([36,43],
             [122,82],
             [122,93],
             [122,102],
             [x5,y5])
    x1,y1 = sizes[0]
    x2,y2 = sizes[1]
    x3,y3 = sizes[2]
    x4,y4 = sizes[3]
    #Measure corners of bounding box in MS paint (pixel just AFTER las pixel of sprite)
    
    strips = [
              
#    SpriteStripAnim('data\PlayerExplode.bmp', (0,1,x1,y1), 7, 1, True, frames),
#    SpriteStripAnim('data\MLRS_Chassis.bmp', (9,4,x2,y2), 4, 1, True, frames),
#    SpriteStripAnim('data\MLRS_Chassis.bmp', (9,88,x3,y3), 4, 1, True, frames),
#    SpriteStripAnim('data\MLRS_Chassis.bmp', (9,184,x4,y4), 4, 1, True, frames),
    SpriteStripAnim('data\LandAircraftCareer.bmp', (topLeft[0],topLeft[1],x5,y5), 5, 1, True, frames),
    ]
    black = Color('black')
    clock = pygame.time.Clock()
    n = 0
    strips[n].iter()
    image = strips[n].next()
    aka = 1
    while True:
        for e in pygame.event.get():
            if e.type == KEYUP:
                if e.key == K_ESCAPE:
                    sys.exit()
                elif e.key == K_RETURN:
                    n += 1
                    aka = 1
                    if n >= len(strips):
                        n = 0
                    strips[n].iter()
        
        
        surface.fill(black)
        xSz,ySz = sizes[n]
        surface.blit(image, (0,0))
        pygame.display.flip()
        image = strips[n].next()
        image = pygame.transform.scale(image,(xSz*3,ySz*3))
        clock.tick(FPS)
    
def SpriteSheetExample():
    import sys
    from pygame.locals import Color, KEYUP, K_ESCAPE, K_RETURN
     
    surface = pygame.display.set_mode((350,400))
    FPS = 12
    frames = FPS / 12
#    strips = [
#        SpriteStripAnim('Explode1.bmp', (0,0,24,24), 8, 1, True, frames),
#        SpriteStripAnim('Explode2.bmp', (0,0,12,12), 7, 1, True, frames),
#        SpriteStripAnim('Explode3.bmp', (0,0,48,48), 4, 1, True, frames) +
#        SpriteStripAnim('Explode3.bmp', (48,48,48,48), 4, 1, True, frames),
#        SpriteStripAnim('Explode4.bmp', (0,0,24,24), 6, 1, True, frames),
#        SpriteStripAnim('Explode5.bmp', (0,0,48,48), 4, 1, True, frames) +
#        SpriteStripAnim('Explode5.bmp', (48,48,48,48), 4, 1, True, frames),
#    ]
   # __init__(self, filename, rect, count, colorkey=None, loop=False, frames=1)
   # rect: x,y,x+offset,y+offset
    topLeft = [113+1,72+1]
    botRight = [145,96]
    botLeft = [113,96]
    topRight = [143,72]
    width,height = botRight[0] - topLeft[0],botRight[1] - topLeft[1]
    x5 = width
    y5 = height
    
    sizes = ([36,43],
             [122,82],
             [122,93],
             [122,102],
             [x5,y5])
    x1,y1 = sizes[0]
    x2,y2 = sizes[1]
    x3,y3 = sizes[2]
    x4,y4 = sizes[3]
    #Measure corners of bounding box in MS paint (pixel just AFTER las pixel of sprite)
    
    strips = [
              
#    SpriteStripAnim('data\PlayerExplode.bmp', (0,1,x1,y1), 7, 1, True, frames),
#    SpriteStripAnim('data\MLRS_Chassis.bmp', (9,4,x2,y2), 4, 1, True, frames),
#    SpriteStripAnim('data\MLRS_Chassis.bmp', (9,88,x3,y3), 4, 1, True, frames),
#    SpriteStripAnim('data\MLRS_Chassis.bmp', (9,184,x4,y4), 4, 1, True, frames),
    SpriteStripAnim('data\LandAircraftCareer.bmp', (topLeft[0],topLeft[1],x5,y5), 5, 1, True, frames),
    ]
    black = Color('black')
    clock = pygame.time.Clock()
    n = 0
    strips[n].iter()
    image = strips[n].next()
    aka = 1
    while True:
        for e in pygame.event.get():
            if e.type == KEYUP:
                if e.key == K_ESCAPE:
                    sys.exit()
                elif e.key == K_RETURN:
                    n += 1
                    aka = 1
                    if n >= len(strips):
                        n = 0
                    strips[n].iter()
        
        
        surface.fill(black)
        xSz,ySz = sizes[n]
        surface.blit(image, (0,0))
        pygame.display.flip()
        image = strips[n].next()
        image = pygame.transform.scale(image,(xSz*3,ySz*3))
        clock.tick(FPS)

# each type of game object gets an init and an
# update function. the update function is called
# once per frame, and it is when each object should
# change it's current position and state. the Player
# object actually gets a "move" function instead of
# update, since it is passed extra information about
# the keyboard
class Particle():
    mass = 0
    radius = 0
    drag_Coef = 0
    
    position = (0,0)
    velocity = (0,0)
    acceleration = (0,0)
    
    
    
    def __init__(self,mass,radius,InitialConditions):
        self.mass = properties[0]
        self.radius = properties[1]
        
    def update(self):
        
class Player(pygame.sprite.Sprite):
    speed = 10
    bounce = 24
    gun_offset = -11
    images = []
    def __init__(self):
        pygame.sprite.Sprite.__init__(self, self.containers)
        self.image = self.images[0]
        self.rect = self.image.get_rect(midbottom=SCREENRECT.midbottom)
        self.reloading = 0
        self.origtop = self.rect.top
        self.facing = -1

    def move(self, direction):
        if direction: self.facing = direction
        self.rect.move_ip(direction*self.speed, 0)
        self.rect = self.rect.clamp(SCREENRECT)
        if direction < 0:
            self.image = self.images[0]
        elif direction > 0:
            self.image = self.images[1]
        self.rect.top = self.origtop - (self.rect.left//self.bounce%2)

    def gunpos(self):
        pos = self.facing*self.gun_offset + self.rect.centerx
        return pos, self.rect.top


class Alien(pygame.sprite.Sprite):
    speed = 13
    animcycle = 12
    images = []
    def __init__(self):
        pygame.sprite.Sprite.__init__(self, self.containers)
        self.image = self.images[0]
        self.rect = self.image.get_rect()
        self.facing = random.choice((-1,1)) * Alien.speed
        self.frame = 0
        if self.facing < 0:
            self.rect.right = SCREENRECT.right

    def update(self):
        self.rect.move_ip(self.facing, 0)
        if not SCREENRECT.contains(self.rect):
            self.facing = -self.facing;
            self.rect.top = self.rect.bottom + 1
            self.rect = self.rect.clamp(SCREENRECT)
        self.frame = self.frame + 1
        self.image = self.images[self.frame//self.animcycle%3]


class Explosion(pygame.sprite.Sprite):
    defaultlife = 12
    animcycle = 3
    images = []
    def __init__(self, actor):
        pygame.sprite.Sprite.__init__(self, self.containers)
        self.image = self.images[0]
        self.rect = self.image.get_rect(center=actor.rect.center)
        self.life = self.defaultlife

    def update(self):
        self.life = self.life - 1
        self.image = self.images[self.life//self.animcycle%2]
        if self.life <= 0: self.kill()


class Shot(pygame.sprite.Sprite):
    speed = -11
    images = []
    def __init__(self, pos):
        pygame.sprite.Sprite.__init__(self, self.containers)
        self.image = self.images[0]
        self.rect = self.image.get_rect(midbottom=pos)

    def update(self):
        self.rect.move_ip(0, self.speed)
        if self.rect.top <= 0:
            self.kill()


class Bomb(pygame.sprite.Sprite):
    speed = 9
    images = []
    def __init__(self, alien):
        pygame.sprite.Sprite.__init__(self, self.containers)
        self.image = self.images[0]
        self.rect = self.image.get_rect(midbottom=
                    alien.rect.move(0,5).midbottom)

    def update(self):
        self.rect.move_ip(0, self.speed)
        if self.rect.bottom >= 470:
            Explosion(self)
            self.kill()


class Score(pygame.sprite.Sprite):
    def __init__(self):
        pygame.sprite.Sprite.__init__(self)
        self.font = pygame.font.Font(None, 20)
        self.font.set_italic(1)
        self.color = Color('white')
        self.lastscore = -1
        self.update()
        self.rect = self.image.get_rect().move(10, 450)

    def update(self):
        if SCORE != self.lastscore:
            self.lastscore = SCORE
            msg = "Score: %d" % SCORE
            self.image = self.font.render(msg, 0, self.color)

def mainMod():
    x = 1
    
def main(winstyle = 0):
    # Initialize pygame
    pygame.init()
    if pygame.mixer and not pygame.mixer.get_init():
        print ('Warning, no sound')
        pygame.mixer = None

    # Set the display mode
    winstyle = 0  # |FULLSCREEN
    bestdepth = pygame.display.mode_ok(SCREENRECT.size, winstyle, 32)
    screen = pygame.display.set_mode(SCREENRECT.size, winstyle, bestdepth)

    #Load images, assign to sprite classes
    #(do this before the classes are used, after screen setup)
    img = load_image('player1.gif')
    Player.images = [img, pygame.transform.flip(img, 1, 0)]
    img = load_image('explosion1.gif')
    Explosion.images = [img, pygame.transform.flip(img, 1, 1)]
    Alien.images = load_images('alien1.gif', 'alien2.gif', 'alien3.gif')
    Bomb.images = [load_image('bomb.gif')]
    Shot.images = [load_image('shot.gif')]

    #decorate the game window
    icon = pygame.transform.scale(Alien.images[0], (32, 32))
    pygame.display.set_icon(icon)
    pygame.display.set_caption('Pygame Aliens')
    pygame.mouse.set_visible(0)

    #create the background, tile the bgd image
    bgdtile = load_image('background.gif')
    background = pygame.Surface(SCREENRECT.size)
    for x in range(0, SCREENRECT.width, bgdtile.get_width()):
        background.blit(bgdtile, (x, 0))
    screen.blit(background, (0,0))
    pygame.display.flip()

    #load the sound effects
    boom_sound = load_sound('boom.wav')
    shoot_sound = load_sound('car_door.wav')
    if pygame.mixer:
        music = os.path.join(main_dir, 'data', 'house_lo.wav')
        pygame.mixer.music.load(music)
        pygame.mixer.music.play(-1)

    # Initialize Game Groups
    aliens = pygame.sprite.Group()
    shots = pygame.sprite.Group()
    bombs = pygame.sprite.Group()
    all = pygame.sprite.RenderUpdates()
    lastalien = pygame.sprite.GroupSingle()

    #assign default groups to each sprite class
    Player.containers = all
    Alien.containers = aliens, all, lastalien
    Shot.containers = shots, all
    Bomb.containers = bombs, all
    Explosion.containers = all
    Score.containers = all

    #Create Some Starting Values
    global score
    alienreload = ALIEN_RELOAD
    kills = 0
    clock = pygame.time.Clock()

    #initialize our starting sprites
    global SCORE
    player = Player()
    Alien() #note, this 'lives' because it goes into a sprite group
    if pygame.font:
        all.add(Score())


    while player.alive():

        #get input
        for event in pygame.event.get():
            if event.type == QUIT or \
                (event.type == KEYDOWN and event.key == K_ESCAPE):
                    return
        keystate = pygame.key.get_pressed()

        # clear/erase the last drawn sprites
        all.clear(screen, background)

        #update all the sprites
        all.update()

        #handle player input
        direction = keystate[K_RIGHT] - keystate[K_LEFT]
        player.move(direction)
        firing = keystate[K_SPACE]
        if not player.reloading and firing and len(shots) < MAX_SHOTS:
            Shot(player.gunpos())
            shoot_sound.play()
        player.reloading = firing

        # Create new alien
        if alienreload:
            alienreload = alienreload - 1
        elif not int(random.random() * ALIEN_ODDS):
            Alien()
            alienreload = ALIEN_RELOAD

        # Drop bombs
        if lastalien and not int(random.random() * BOMB_ODDS):
            Bomb(lastalien.sprite)

        # Detect collisions
        for alien in pygame.sprite.spritecollide(player, aliens, 1):
            boom_sound.play()
            Explosion(alien)
            Explosion(player)
            SCORE = SCORE + 1
            player.kill()

        for alien in pygame.sprite.groupcollide(shots, aliens, 1, 1).keys():
            boom_sound.play()
            Explosion(alien)
            SCORE = SCORE + 1

        for bomb in pygame.sprite.spritecollide(player, bombs, 1):
            boom_sound.play()
            Explosion(player)
            Explosion(bomb)
            player.kill()

        #draw the scene
        dirty = all.draw(screen)
        pygame.display.update(dirty)

        #cap the framerate
        clock.tick(40)

    if pygame.mixer:
        pygame.mixer.music.fadeout(1000)
    pygame.time.wait(1000)
    pygame.quit()


play = 'sprites'
#call the "main" function if running this script
if __name__ == '__main__': 
    if play=='Aliens':
        main()
    elif play=='sprites':
        SpriteSheetExample()
    elif play=='AliensSprites':
        mainMod()


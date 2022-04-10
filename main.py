from math import pi, sin, cos, atan2, radians, degrees
from random import randint
import pygame as pg
import pygame_gui as pgui


FLLSCRN = True          #True for Fullscreen, or False for Window
BOIDZ = 200             #How many boids to spawn, too many may slow fps
WRAP = True             #False avoids edges, True wraps to other side
SPEED = 150             #Movement speed
WIDTH = 1200            #Window Width (1200)
HEIGHT = 800            #Window Height (800)
BGCOLOR = (0, 0, 0)     #Background color in RGB
FPS = 60                #30-90
SHOWFPS = True          #Frame rate debug


class Boid(pg.sprite.Sprite):

    def __init__(self, grid, drawSurf, isFish=False):  #cHSV=None
        super().__init__()  #Calling the Sprite class’s __init__ method
        self.grid = grid  #Associating the boid with the spatial grid on the screen
        self.drawSurf = drawSurf  #The layer on which the boid is drawn, the screen
        self.image = pg.Surface((15, 15)).convert()  #15x15 pixel surface is created, and is converted to an image
        self.image.set_colorkey(0)  #Giving color to each sprite
        self.color = pg.Color(0)  #Preps color so we can use HSVA
        self.color.hsva = (randint(0,360), 90, 90) #Setting the color
        pg.draw.polygon(self.image, self.color, ((7,0), (13,14), (7,11), (1,14), (7,0)))
        #Creating the arrow like shape of each individual boid
        #The coordinates are of the vertices
        self.bSize = 17
        #The closest that the neighbouring boids can get to the current boid without it moving away from its neighbours
        self.orig_image = pg.transform.rotate(self.image.copy(), -90)
        #Mirroring the half arrow that we earlier created to form a full image
        self.dir = pg.Vector2(1, 0)  #Makes all boids spawn facing forwards
        maxW, maxH = self.drawSurf.get_size()
        self.rect = self.image.get_rect(center=(randint(50, maxW - 50), randint(50, maxH - 50)))
        #Creating a rectangle using maxW and maxH
        #Although the boids look like arrows, it is this rectangle that is actually dealing with all of the boid collisions and movements
        self.ang = randint(0, 360)  #Random start angle and position
        self.pos = pg.Vector2(self.rect.center)  #Stores the centre of the rectangle
        self.grid_lastpos = self.grid.getcell(self.pos)  #Stores the last known position of the boid
        self.grid.add(self, self.grid_lastpos)  #Adds the last known position of the boid to the spatial grid

    def update(self, dt, speed, coherence, avoidance, adherence, ejWrap=False):
        maxW, maxH = self.drawSurf.get_size()
        selfCenter = pg.Vector2(self.rect.center)  #Centre of boid saved
        turnDir = xvt = yvt = yat = xat = 0
        turnRate = 120 * dt  #Responsible for how smoothly the boids turn
        margin = 42
        self.ang = self.ang + randint(-4, 4)
        #Makes it look as though the boid is looking around before it moves
        self.grid_pos = self.grid.getcell(self.pos)  #Grid position returned
        if self.grid_pos != self.grid_lastpos:  #If they are equal, updation not required
            self.grid.add(self, self.grid_pos)  #Added to new position
            self.grid.remove(self, self.grid_lastpos)  #Removed from old position
            self.grid_lastpos = self.grid_pos  #Last known position updated to match the boid's current position
        #Get nearby boids and sort them by order of their distances to the current boid
        near_boids = self.grid.getnear(self, self.grid_pos)
        neiboids = sorted(near_boids, key=lambda i: pg.Vector2(i.rect.center).distance_to(selfCenter))
        del neiboids[7:]  #Keep the 7 closest, dump the rest
        #This is because we only want our boid to be influenced by the 7 closest boids to it, and not EVERY boid on the screen
        #When the boid has neighbours
        if (ncount := len(neiboids)) > 1:
            nearestBoid = pg.Vector2(neiboids[0].rect.center)
            for nBoid in neiboids:  #Adds up neighbor vectors & angles for averaging
                xvt += nBoid.rect.centerx
                yvt += nBoid.rect.centery
                yat += sin(radians(nBoid.ang))
                xat += cos(radians(nBoid.ang))
            tAvejAng = degrees(atan2(yat, xat))
            targetV = (xvt / ncount, yvt / ncount)  #Position boid wants to go to, average position of all boids
            #If too close, move away from closest neighbor
            if selfCenter.distance_to(nearestBoid) < self.bSize + avoidance : targetV = nearestBoid
            tDiff = targetV - selfCenter  #Get angle differences for steering
            tDistance, tAngle = pg.math.Vector2.as_polar(tDiff)
            #If the boid is close enough to neighbors, match their average angle
            if tDistance < self.bSize*5 + coherence*2 : tAngle = tAvejAng
            #Computes the difference to reach target angle for smooth steering
            angleDiff = (tAngle - self.ang) + 180
            if abs(tAngle - self.ang) > .5 + ((adherence * 5) / 1000): turnDir = (angleDiff / 360 - (angleDiff // 360)) * 360 - 180
            #If the boid gets too close to the target, steer away
            if tDistance < self.bSize + avoidance and targetV == nearestBoid : turnDir = -turnDir
            #We take the negative of the current turnDir so the boid moves away from its nearest neighbour (^)
        #Avoid edges of screen by turning toward the edge normal-angle
        sc_x, sc_y = self.rect.centerx, self.rect.centery
        #If wrap is disabled, we need to make the boids stay within the frame
        if not ejWrap and min(sc_x, sc_y, maxW - sc_x, maxH - sc_y) < margin:
            if sc_x < margin : tAngle = 0  #If on the right, go left
            elif sc_x > maxW - margin : tAngle = 180  #If on the left, go right
            if sc_y < margin : tAngle = 90  #If at the bottom, go up
            elif sc_y > maxH - margin : tAngle = 270    #If at the top, go down
            angleDiff = (tAngle - self.ang) + 180  #Increase turnRate to keep the boids on screen
            turnDir = (angleDiff / 360 - (angleDiff // 360)) * 360 - 180
            edgeDist = min(sc_x, sc_y, maxW - sc_x, maxH - sc_y)
            turnRate = turnRate + (1 - edgeDist / margin) * (20 - turnRate) #turnRate=minRate, 20=maxRate
        if turnDir != 0:  #Steers based on turnDir, handles left or right
            self.ang += turnRate * abs(turnDir) / turnDir
        self.ang %= 360  #Ensures that the angle stays within 0-360
        #Now we make the image of the boid match all the updates that we made to the boid container rectangle
        self.image = pg.transform.rotate(self.orig_image, -self.ang)
        self.rect = self.image.get_rect(center=self.rect.center)  #Recentering fix
        self.dir = pg.Vector2(1, 0).rotate(self.ang).normalize()
        self.pos += self.dir * dt * (speed + (7 - ncount) * 5)  #Movement speed
        #Optional screen wrap
        #We now update the boids reappear at the opposite side of the screen
        if ejWrap and not self.drawSurf.get_rect().contains(self.rect):
            if self.rect.bottom < 0 : self.pos.y = maxH
            elif self.rect.top > maxH : self.pos.y = 0
            if self.rect.right < 0 : self.pos.x = maxW
            elif self.rect.left > maxW : self.pos.x = 0
        #Actually update the position of the boid
        self.rect.center = self.pos


class BoidGrid():  #Tracks boids in the spatial partition grid

    def __init__(self):
        self.grid_size = 100  #Screen partitioned
        self.dict = {}
    #Finds the grid cell corresponding to given position
    def getcell(self, pos):
        return (pos[0]//self.grid_size, pos[1]//self.grid_size)
    #Boids add themselves to cells when crossing into new cell
    def add(self, boid, key):
        if key in self.dict:
            self.dict[key].append(boid)
        else:
            self.dict[key] = [boid]
    #They also remove themselves from the previous cell
    def remove(self, boid, key):
        if key in self.dict and boid in self.dict[key]:
            self.dict[key].remove(boid)
    #Returns a list of nearby boids within all surrounding 9 cells
    def getnear(self, boid, key):
        if key in self.dict:
            nearby = []
            for x in (-1, 0, 1):
                for y in (-1, 0, 1):
                    nearby += self.dict.get((key[0] + x, key[1] + y), [])
            nearby.remove(boid)
        return nearby


def main():
    pg.init()  #Preparing the window
    pg.display.set_caption("PyNBoids")
    if FLLSCRN:
        #Getting the display size
        currentRez = (pg.display.Info().current_w, pg.display.Info().current_h)
        screen = pg.display.set_mode(currentRez, pg.SCALED | pg.NOFRAME | pg.FULLSCREEN, vsync=1)
        #pg.mouse.set_visible(False)
    else: screen = pg.display.set_mode((WIDTH, HEIGHT), pg.RESIZABLE | pg.SCALED, vsync=1)
    max_width, max_height = screen.get_size()
    #Handles all of the UI rendering from pygame_gui
    manager = pgui.UIManager((WIDTH, HEIGHT))

    #Slider to control cohesion
    cohesion_slider = pgui.elements.UIHorizontalSlider(
        pg.Rect((10, 30), (200, 20)),
        0,
        (0, 100),
        manager=manager
    )
    cohesion_slider.enable()  #To make it interactive

    #Slider to control avoidance
    avoidance_slider = pgui.elements.UIHorizontalSlider(
        pg.Rect((10, 60), (200, 20)),
        0,
        (0, 100),
        manager=manager
    )
    avoidance_slider.enable()  #To make it interactive

    #Slider to control adherence
    adherence_slider = pgui.elements.UIHorizontalSlider(
        pg.Rect((10, 90), (200, 20)),
        0,
        (0, 100),
        manager=manager
    )
    adherence_slider.enable()  #To make it interactive

    #Slider to control the speed of the boids
    speed_slider = pgui.elements.UIHorizontalSlider(
        pg.Rect((10, 120), (200, 20)),
        0,
        (0, 100),
        manager=manager
    )
    speed_slider.enable()  #To make it interactive

    #Used to either turn wrap on or off
    wrap_toggle = pgui.elements.UIButton(
        pg.Rect((10, 150), (100, 40)),
        text="Wrap Around",
        manager=manager
    )

    #Used to display the number of frames per second
    fps_toggle = pgui.elements.UIButton(
        pg.Rect((10, 200), (100, 40)),  #Location and dimensions on screen
        text="Show FPS",
        manager=manager
    )

    boidTracker = BoidGrid()
    nBoids = pg.sprite.Group()
    #Creating the group of boids to be rendered in the UI
    #Spawns the desired number of boids
    for n in range(BOIDZ) : nBoids.add(Boid(boidTracker, screen))

    if SHOWFPS: font = pg.font.Font(None, 30)
    clock = pg.time.Clock()
    coherence = avoidance = adherence = speed = 0
    wrap = fps = False
    #Main loop
    while True:
        for e in pg.event.get():
            if e.type == pg.QUIT or e.type == pg.KEYDOWN and (e.key == pg.K_ESCAPE or e.key == pg.K_q or e.key==pg.K_SPACE):
            #Condition to break out of the loop
                return
            #The main user event to control the various factors using the sliders
            if e.type == pg.USEREVENT:
                if e.user_type == pgui.UI_HORIZONTAL_SLIDER_MOVED:
                    if e.ui_element == cohesion_slider:
                        coherence = e.value  #Changing the value of coherence
                    elif e.ui_element == avoidance_slider:
                        avoidance = e.value  #Changing the value of avoidance
                    elif e.ui_element == adherence_slider:
                        adherence = e.value  #Changing the value of adherence
                    elif e.ui_element == speed_slider:
                        speed = e.value  #Changing the value of speed
                elif e.user_type == pgui.UI_BUTTON_PRESSED:
                    if e.ui_element == wrap_toggle:
                        wrap = not wrap  #Toggling wrap value based on previous state
                    if e.ui_element == fps_toggle:
                        fps = not fps

            manager.process_events(e)  #Processing e and implementing it in the UI

        dt = clock.tick(FPS) / 1000
        #Keep checking after dt intervals to detect changes in UI to update the boids
        manager.update(dt)
        screen.fill(BGCOLOR)
        #Update boid logic, then draw them
        nBoids.update(dt, SPEED + speed, coherence, avoidance, adherence, wrap)
        nBoids.draw(screen)
        manager.draw_ui(screen)
        #If true, displays the FPS in the upper left corner for debugging
        if fps : screen.blit(font.render(str(int(clock.get_fps())), True, [0, 200, 0]), (8, 8))
        screen.blit(font.render("Coherence Factor", True, [0, 200, 0]), (220, 30))
        screen.blit(font.render("Avoidance Factor", True, [200, 0, 0]), (220, 60))
        screen.blit(font.render("Adherence Factor", True, [0, 0, 200]), (220, 90))
        screen.blit(font.render("Speed", True, [100, 200, 100]), (220, 120))
        pg.display.update()

if __name__ == '__main__':
    main()  
    pg.quit()
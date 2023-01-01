'''A Python Class to implement a very basic Pygame Plotting
Arena. Plot various objects, animate them, scaling, dealing
with mouse inputs, Updating etc'''


import pygame, sys, math
from pygame.locals import *
import Sprites as spr
import random

class PyArena:
    def __init__(self, w = 800,h = 800, bgColor = "Grey", axisColor = "Grey", TIMER_DELAY = 30):
        '''Create a Pygame Arena, TIMER_DELAY sets the freq of screen updating in milliseconds'''
        self.w = w
        self.cx = w//2
        self.h = h
        self.cy = h//2
        self.bgColor = bgColor # the background color
        self.axisColor = axisColor # Color of the axes
        self.delay = TIMER_DELAY # The screen refresh  delay im milliseconds 
        self.screen = pygame.display.set_mode((self.w, self.h))
        self.clock = pygame.time.Clock()
        pygame.time.set_timer(USEREVENT+1, TIMER_DELAY) # Start the timer, assign it a USEREVENT
        self.counter = 0 # Init a counter to keep track of time passed
        self.obstacles = [] # Empty list to hold all the sprites
        self.robots = []
        self.goals = []
        self.swarming = False # One robot or Swarm?
        self.manualBotID = 0 # Which bot is currently under manual control

    def ReDraw(self):
        '''ReDraw on the Objects on the Screen'''
        
        self.screen.fill(pygame.Color(self.bgColor)) # Clear the screen
    
        #draw the axes
        pygame.draw.lines(self.screen, pygame.Color(self.axisColor), False, ((self.cx,0),(self.cx,self.h))) # Y Axis
        pygame.draw.lines(self.screen, pygame.Color(self.axisColor), False, ((0,self.cy),(self.w,self.cy))) # X Axis

        # Update and then Draw all the sprites
        for sprites in self.obstacles, self.goals, self.robots:

            for sprite in sprites:

                if self.counter % 7 == 0: #Sprites update every 00 ms
                    sprite.Update(self.obstacles) 
                    ''' Update all the sprites,  Fixed sprites 
                    do nothing during Update(). Robots may need to re-position, scan for targets,
                    zombie sprites may just move a little...'''
            
                sprite.PygameDraw(self.screen) 
                ''' Each sprite must be able to draw itself onto the screen
                Rather than sprites passing info to this class, and implementing
                the draw function here, it's simpler if every sprite knows 
                how to draw itself. Thus we can have different sub-classes of sprites
                with different drawing attributes, and we can add
                more sub-classes in the future that implement other drawing
                attributes, without having to change any of the code here'''

        #After everything is re-drawn, update the screen

        pygame.display.update()
       
    def GetManualBot(self):
        '''Returns the currently active robot under
        manual control'''

        return self.robots[self.manualBotID]

    def ObeyRobotCommands(self,event):
        '''Deal with any User genrated Robot Commands'''
        bot = self.GetManualBot() # Find the robot currently under manual control
        if event.key == pygame.K_UP: # UP Arrow pressed, step robot forwards
            bot.StepForward(1)
        if event.key == pygame.K_DOWN: # Down arrow pressed, step robot backwards
            bot.StepBackward(1)
        if event.key == pygame.K_LEFT: #Left arrow pressed turn left
            bot.TurnLeft()
        if event.key == pygame.K_RIGHT: # Right arrow pressed turn right
            bot.TurnRight()
        if event.key == pygame.K_a: # Toggle the robot's auto mode
            bot.ToggleAutoControl()

    def AddObstacles(self, event):
        '''Add any user genrated obstacles'''
        
        if event.key == pygame.K_o: # On pressing 'O' key, an obstacle is added at the mouse curcor
            pos = self.WH2XY(pygame.mouse.get_pos())
            nObs = len(self.obstacles) # New sprite will have ID = len(obstacles) + 1
            self.obstacles.append(spr.Sprite(nObs + 1, pos)) # Add an obstacle at the mouse position

            # Lines below added for debugging pruposes
            '''print("Creating....,")
            for s in self.obstacles:
                print(s.pos)'''


    def  Add_MoveGoal(self, event):
        '''Move the goal or if in swarming mode, add more goals'''

        if event.key == pygame.K_g: # On pressing 'G' key, the goal is moved to the mouse cursor posn
            pos = self.WH2XY(pygame.mouse.get_pos())
            if self.swarming == True: #multiple robots and goals
                pass
            else:
                self.goals[0].SetPos(pos)
                self.robots[0].SetGoal(pos) # Single robot single goal seeking
                self.robots[0].SetSpeed(5) # Get the robot moving
                #self.goals[1].SetPos(pos)
                #self.robots[1].SetGoal(pos) # Single robot single goal seeking
                #self.robots[1].SetSpeed(5) # Get the robot moving

    def Update(self):
        '''Perform all mandatory repeated tasks'''
       #Deal with user inputs

        for event in pygame.event.get():
            if event.type == USEREVENT+1: #Timer has ticked
                self.counter += 1 # Increment the counter
                # Re-Draw the screen
                self.ReDraw() # ReDraw will also update ALL sprites

            if event.type == pygame.QUIT or (
                    event.type == pygame.KEYDOWN and event.key == pygame.K_q):  # USER CLOSES PROGRAM
                pygame.quit()
                sys.exit()
               
            if event.type == pygame.KEYDOWN: # Deal with User Commands
                
                #Obey User Robot Commands
                self.ObeyRobotCommands(event)

                #Obey User Goal Commands
                self.Add_MoveGoal(event)

                #Add User Generated Obstacles
                self.AddObstacles(event)
            
    def WH2XY(self,screenWH):
        '''Convert a pygame Screen WH coordinates to Coordinate axis
        XY value tuple'''

        return (screenWH[0] - self.cx, self.cy - screenWH[1])

    def AddDebugObstacles(self, t):
        '''Create a bunch of obstacles for debugging purposes'''
        for pos in t:
            nObs = len(self.obstacles) # New sprite will have ID = len(obstacles) + 1
            self.obstacles.append(spr.Sprite(nObs + 1, pos)) # Add an obstacle at the mouse position

if __name__ == "__main__":
    arena1 = PyArena() # Create an Arena
    
    wmax, hmax = arena1.screen.get_width(), arena1.screen.get_height()
    
    #Create a goal
    pos = arena1.WH2XY((400,700))
    goal1 = spr.Sprite(500, pos)
    goal1.SetImage("images/goal1.png")
    arena1.goals.append(goal1)

    # Create a robot
    pos = arena1.WH2XY((100, 100))
    rob1 = spr.GoalSeeker(1000, pos)
    rob1.SetImage("images/rover1.png")
    rob1.SetHeading(random.randint(0,360))
    rob1.SetSpeed(5)
    rob1.SetSensor(300, 70)
    rob1.SetGoal(goal1.GetPos())
    arena1.robots.append(rob1)

    '''#Create a goal
    pos2 = arena1.WH2XY((400,700))
    goal2 = spr.Sprite(500, pos2)
    goal2.SetImage("images/goal1.png")
    arena1.goals.append(goal2)'''

    # Create a robot
    '''pos2 = arena1.WH2XY((200, 200))
    rob2 = spr.GoalSeeker(1000, pos2)
    rob2.SetImage("images/rover1.png")
    rob2.SetHeading(random.randint(0,360))
    rob2.SetSpeed(5)
    rob2.SetSensor(300, 70)
    rob2.SetGoal(goal2.GetPos())
    arena1.robots.append(rob2)'''

    #Create Some obstacles

    '''trap1 = [(-271, 86), (-272, 84), (-259, 51), (-228, 7), (-246, 27), (-197, -29), (-207, -6), (-185, -66)
            , (-133, -81), (-166, -70), (-94, -84), (-58, -76), (-39, -70), (-36, -60), (-26, -27)
            , (-9, 3), (-1, 25), (-1, 36), (-10, 79), (-20, 113), (-32, 150), (-41, 190), (-41, 217) ,(-4, 68)]

    arena1.AddDebugObstacles(trap1)'''
      
    while True:
        arena1.Update()


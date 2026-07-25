#!/usr/bin/python3

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

# The GPL3 license is a widely used free software license that allows 
# users to run, study, share, and modify software. It requires that 
# any derivative works or modifications made to the software be 
# licensed under the GPL3 and that the source code be made available 
# to anyone who receives the software.

import math
import os
import sys
import numpy as np
import abc
from typing import List, Type, Dict, Callable
import pygame

from turtlesim_plus.world import (
    WORLD_SIZE, SCREEN_SIZE, world_to_screen, clamp_to_world, world_length_to_screen
)

class Entity(abc.ABC):
    def __init__(self,name: str) -> None:
        self.name = name
class GraphicsEntity(Entity):
    def __init__(self, name: str):
        super().__init__(name=name)
        self.pose = [0.0,0.0,0.0]
        self.size = None
        self.graphics = None
    def set_pose(self,pose:List[float]):
        self.pose = pose
    @abc.abstractmethod
    def render(self,screen):
        pass
    def set_graphics(self,graphics):
        self.graphics = graphics
class PhysicsEntity(abc.ABC):
    def __init__(self, name:str):
        super().__init__(name=name)
        self.state = None
    @abc.abstractmethod
    def set_state(self,state:List[float]):
        self.state = state
    @abc.abstractmethod
    def update(self,*args, **kwargs):
        pass
class Pizza(GraphicsEntity):  
    def __init__(self, name: str,pose:List[float]):
        super().__init__(name)
        self.pose = pose
    def render(self,screen):
        img = self.graphics
        size = np.min([math.floor(img.get_size()[0]/2),math.floor(img.get_size()[1]/2)])
        sx,sy = world_to_screen(self.pose[0],self.pose[1])
        screen.blit(img,(sx-size,sy-size))

class Parcel(GraphicsEntity):
    # parcel.png ships at 87x88px, roughly double the other sprites (pizza.png is
    # 36x36, turtle sprites are ~45-50px) -- scale it down to match on load so it
    # doesn't dominate the scene.
    TARGET_SIZE = 36
    def __init__(self, name: str,pose:List[float]):
        super().__init__(name)
        self.pose = pose
    def set_graphics(self,graphics):
        # parcel.png ships as a flat, fully-opaque image with a solid white
        # background (no alpha channel at all, unlike pizza.png) -- punch the
        # white out to a real per-pixel alpha of 0 before scaling, otherwise it
        # renders as a white square block instead of just the icon outline.
        graphics = graphics.convert_alpha()
        rgb = pygame.surfarray.pixels3d(graphics)
        alpha = pygame.surfarray.pixels_alpha(graphics)
        white = (rgb[:,:,0]==255) & (rgb[:,:,1]==255) & (rgb[:,:,2]==255)
        alpha[white] = 0
        del rgb,alpha
        w,h = graphics.get_size()
        scale = self.TARGET_SIZE/max(w,h)
        scaled = pygame.transform.smoothscale(graphics,(max(1,round(w*scale)),max(1,round(h*scale))))
        super().set_graphics(graphics=scaled)
    def render(self,screen):
        img = self.graphics
        size = np.min([math.floor(img.get_size()[0]/2),math.floor(img.get_size()[1]/2)])
        sx,sy = world_to_screen(self.pose[0],self.pose[1])
        screen.blit(img,(sx-size,sy-size))

class DropZone(GraphicsEntity):
    """Static, non-physics marker for where a carried parcel must be dropped off."""
    def __init__(self, name: str,pose:List[float],radius:float,color=(255,255,0,80)):
        super().__init__(name)
        self.pose = pose
        self.radius = radius
        self.color = color
    def render(self,screen):
        cx,cy = world_to_screen(self.pose[0],self.pose[1])
        r_px = max(1,int(world_length_to_screen(self.radius)))
        circle_surf = pygame.Surface((r_px*2,r_px*2),pygame.SRCALPHA)
        pygame.draw.circle(circle_surf,self.color,(r_px,r_px),r_px)
        screen.blit(circle_surf,(cx-r_px,cy-r_px))

class Turtle(PhysicsEntity,GraphicsEntity):
    def __init__(self, name: str,init_pose:List[float]=[0.0,0.0,0.0]):
        PhysicsEntity.__init__(self,name)
        self.pose = init_pose
        self.state = init_pose
        # Pen trail: on by default (matches stock turtlesim). A "stroke" is a run of
        # connected points; teleporting or toggling the pen back on starts a new
        # stroke so the line doesn't connect across a jump or a pen-up gap.
        self.pen_on = True
        self.pen_color = (255,255,255)
        self.pen_width = 3
        self.trail = [[tuple(init_pose[0:2])]]
    def set_pose(self, pose: List[float]):
        super().set_pose(pose=pose)
        self.state = pose
    def set_state(self, state:List[float]):
        super().set_state(state=state)
        self.set_pose(state)
    def set_pen(self,r:int=255,g:int=255,b:int=255,width:int=3,off:bool=False):
        self.pen_color = (r,g,b)
        self.pen_width = max(1,width)
        pen_was_off = not self.pen_on
        self.pen_on = not off
        if pen_was_off and self.pen_on:
            self.trail.append([tuple(self.pose[0:2])])
    def clear_trail(self):
        self.trail = [[tuple(self.pose[0:2])]]
    def teleport(self,x:float,y:float,theta:float):
        x,y = clamp_to_world(x,y)
        self.pose = [float(x),float(y),theta]
        self.state = self.pose
        if self.pen_on:
            self.trail.append([tuple(self.pose[0:2])])
    def update(self,cmd_vel:List[float]=[0.0,0.0],dt:float=0.1):
        # accurate when velocity is constant
        def expm(theta):
            q = np.array([math.cos(theta),math.sin(theta)])
            return q
        dtheta = cmd_vel[1]*dt
        q_dtheta = expm(dtheta)
        dR = np.array([q_dtheta,[-q_dtheta[1],q_dtheta[0]]]).T
        q = expm(self.pose[2])
        q_new = dR@q
        theta_new = math.atan2(q_new[1],q_new[0])
        if  abs(dtheta)<0.00000001:
            V = np.eye(2)*dt
        else:
            # V is the integral of R(omega*t) dt over [0,dt], parameterized by
            # dtheta=omega*dt -- it needs a dt/dtheta factor to have units of
            # time (matching the dt*I above); without it, V (and therefore the
            # resulting displacement) shrinks to ~0 as dtheta shrinks toward
            # (but doesn't reach) the threshold above, instead of smoothly
            # matching the straight-line case. That silently froze the turtle
            # in place for any cmd_vel with a very small but nonzero angular.z
            # (e.g. a nearly-but-not-quite-aligned steering controller).
            V = (dt/dtheta)*np.array([[q_dtheta[1],q_dtheta[0]-1],[1-q_dtheta[0],q_dtheta[1]]])
        R_new = np.array([q_new,[-q_new[1],q_new[0]]]).T
        dp = R_new@V@np.array([cmd_vel[0],0])
        p_new = np.array(self.pose[0:2])+dp

        x_new,y_new = clamp_to_world(p_new[0],p_new[1])
        self.pose = [float(x_new),float(y_new),theta_new]

        self.state = self.pose
        if self.pen_on:
            # Only record a point when the position actually changed -- update()
            # runs every physics tick (100Hz) regardless of velocity, so a
            # stationary turtle would otherwise pile up thousands of coincident
            # points at the same spot, which pygame.draw.lines renders as a
            # solid blob (visible as a stray dot in the turtle's pen color).
            new_point = tuple(self.pose[0:2])
            if self.trail[-1][-1] != new_point:
                self.trail[-1].append(new_point)
    def render(self,screen):
        for stroke in self.trail:
            if len(stroke)>=2:
                screen_points = [world_to_screen(x,y) for x,y in stroke]
                pygame.draw.lines(screen,self.pen_color,False,screen_points,self.pen_width)

        img = pygame.transform.rotate(self.graphics, self.pose[2]*180/math.pi-90)

        size = np.min([math.floor(img.get_size()[0]/2),math.floor(img.get_size()[1]/2)])
        sx,sy = world_to_screen(self.pose[0],self.pose[1])
        screen.blit(img,(sx-size,sy-size))
class ScannerData():
        def __init__(self,type,angle,distance):
            self.type = type
            self.angle = angle
            self.distance = distance
def draw_polygon_alpha(surface, color, points):
    lx, ly = zip(*points)
    min_x, min_y, max_x, max_y = min(lx), min(ly), max(lx), max(ly)
    target_rect = pygame.Rect(min_x, min_y, max_x - min_x, max_y - min_y)
    shape_surf = pygame.Surface(target_rect.size, pygame.SRCALPHA)
    pygame.draw.polygon(shape_surf, color, [(x - min_x, y - min_y) for x, y in points])
    surface.blit(shape_surf, target_rect)
class Scanner():
    def __init__(self, radius:float=4.0,range:float=math.pi/3,color=(255,0,0,127)):
        self.radius = radius
        self.range = range # 
        self.detection_types = []
        self.color = color
    def add_detection_type(self,type:Type[Entity]):
        if not type in self.detection_types:
            self.detection_types.append(type)
    # Below this separation, the bearing angle to a target is numerically
    # unstable (atan2 of a near-zero vector is dominated by float rounding
    # noise) and can land anywhere -- e.g. a turtle placed via teleport_absolute
    # (a float32 ROS field) sitting "on" a pizza spawned via a float64 service
    # can differ by ~1e-7, enough to swing the computed bearing outside a narrow
    # detection cone even though the target is effectively co-located.
    COINCIDENT_EPSILON = 1e-3
    def measure(self,pose:List[float],turtle,entity_list:List[Entity]):
        scanner_data_array = []
        entity_array = []
        for entity in entity_list.values():
            # Compared by name rather than object identity: `turtle` here is the
            # component's own component object, not the composite entity that
            # appears (under `turtle`'s name) in entity_list.
            # Type filtering was previously disabled (dead code below), so any
            # scanner detected every entity regardless of add_detection_type() --
            # harmless while only Turtle/Pizza existed, but wrong once other
            # always-present entity types (e.g. DropZone) share the world.
            if entity.name != turtle.name and any(isinstance(entity,detection_type) for detection_type in self.detection_types):
                dx = entity.pose[0] - pose[0]
                dy = entity.pose[1] - pose[1]
                distance = math.sqrt(dx*dx + dy*dy)
                angle = math.atan2(dy, dx) - pose[2]

                # normalize the angle to [-pi, pi]
                while angle > math.pi:
                    angle -= 2*math.pi
                while angle < -math.pi:
                    angle += 2*math.pi
                    # check if the entity is within the range of the scanner
                if distance <= self.radius and (distance <= self.COINCIDENT_EPSILON or abs(angle) <= self.range/2):
                    # create a ScannerData object and add it to the list of detected entities
                    scanner_data_array.append(ScannerData(type=type(entity),angle=angle,distance=distance))
                    entity_array.append(entity)
        return scanner_data_array,entity_array
    def render(self,screen,pose):
        origin = world_to_screen(pose[0],pose[1])
        vertices = [origin]
        for i in range(int(math.degrees(-self.range / 2)), int(math.degrees(self.range / 2)) + 1):
            rads = math.radians(i)+pose[2]
            vertices.append(world_to_screen(pose[0] + self.radius * math.cos(rads),pose[1] + self.radius * math.sin(rads)))
        vertices.append(origin)
        #pygame.draw.polygon(screen,pygame.color.Color(255,0,0,a=128), vertices)
        draw_polygon_alpha(screen,self.color,vertices)
class TurtleCommandInterface(PhysicsEntity,GraphicsEntity):
    def __init__(self, turtle:Turtle):
        PhysicsEntity.__init__(self,name=turtle.name)
        self.turtle = turtle
        self.command_velocity = [0.0,0.0]
        self.state = turtle.state
        self.pose = turtle.pose
    def set_pose(self, pose: List[float]):
        PhysicsEntity.set_state(self,state=pose)
        GraphicsEntity.set_pose(self,pose=pose)
        self.turtle.set_pose(pose=pose)
    def set_state(self,state: List[float]):
        PhysicsEntity.set_state(self,state=state)
        GraphicsEntity.set_pose(self,pose=state)
        self.turtle.set_state(state=state)
    def update(self, dt:float=0.1,*args, **kwargs):
        self.turtle.update(cmd_vel=self.command_velocity,dt=dt)
    def render(self,screen):
        self.turtle.render(screen)
class TurtleEatInterface(PhysicsEntity,GraphicsEntity):
    def __init__(self, turtle:Turtle,eat_radius:float=2.0,eat_angle_range:float=math.pi/3):
        PhysicsEntity.__init__(self,name=turtle.name)
        self.turtle = turtle
        self.eat_range = Scanner(radius=eat_radius,range=eat_angle_range,color=(0,255,0,127))
        self.eat_range.add_detection_type(Pizza)
        self.edibles = []
    def set_pose(self, pose: List[float]):
        PhysicsEntity.set_state(self,state=pose)
        GraphicsEntity.set_pose(self,pose=pose)
        self.turtle.set_pose(pose=pose)
    def set_state(self,state: List[float]):
        PhysicsEntity.set_state(self,state=state)
        GraphicsEntity.set_pose(self,pose=state)
        self.turtle.set_state(state=state)
    def update(self, dt:float=0.1,entity_list:Dict[str,Entity] = {},*args, **kwargs):
        edibles_data,self.edibles =  self.eat_range.measure(pose=self.pose,turtle=self,entity_list=entity_list)
    def render(self,screen):
        self.eat_range.render(screen,pose=self.pose)
        self.turtle.render(screen)
class TurtleDeliveryInterface(PhysicsEntity,GraphicsEntity):
    def __init__(self, turtle:Turtle,pickup_radius:float=2.0,pickup_angle_range:float=math.pi/3):
        PhysicsEntity.__init__(self,name=turtle.name)
        self.turtle = turtle
        self.pickup_range = Scanner(radius=pickup_radius,range=pickup_angle_range,color=(0,0,255,127))
        self.pickup_range.add_detection_type(Parcel)
        self.nearby_parcels = []
    def set_pose(self, pose: List[float]):
        PhysicsEntity.set_state(self,state=pose)
        GraphicsEntity.set_pose(self,pose=pose)
        self.turtle.set_pose(pose=pose)
    def set_state(self,state: List[float]):
        PhysicsEntity.set_state(self,state=state)
        GraphicsEntity.set_pose(self,pose=state)
        self.turtle.set_state(state=state)
    def update(self, dt:float=0.1,entity_list:Dict[str,Entity] = {},*args, **kwargs):
        _,self.nearby_parcels = self.pickup_range.measure(pose=self.pose,turtle=self,entity_list=entity_list)
    def render(self,screen):
        self.pickup_range.render(screen,pose=self.pose)
        self.turtle.render(screen)
class TurtleScannerInterface(PhysicsEntity,GraphicsEntity):
    def __init__(self, turtle:Turtle,scanner_radius:float=4.0,scanner_angle_range:float=math.pi/3):
        PhysicsEntity.__init__(self,name=turtle.name)
        self.turtle = turtle
        self.scanner = Scanner(radius=scanner_radius,range=scanner_angle_range)
        self.scanner.add_detection_type(Turtle)
        self.scanner.add_detection_type(Pizza)
        self.scanner.add_detection_type(Parcel)
        self.scanner_output = []
    def set_pose(self, pose: List[float]):
        PhysicsEntity.set_state(self,state=pose)
        GraphicsEntity.set_pose(self,pose=pose)
        self.turtle.set_pose(pose=pose)
    def set_state(self,state: List[float]):
        PhysicsEntity.set_state(self,state=state)
        GraphicsEntity.set_pose(self,pose=state)
        self.turtle.set_state(state=state)
    def update(self, dt:float=0.1,entity_list:Dict[str,Entity] = {},*args, **kwargs):
        self.scanner_output,detected_entity_list =  self.scanner.measure(pose=self.pose,turtle=self,entity_list=entity_list)
    def render(self,screen):
        self.scanner.render(screen,pose=self.pose)
        self.turtle.render(screen)

class EntityManager():
    def __init__(self,entity_type:Type[Entity]):
        self.entity_list = {}
        self.entity_type = entity_type
    def add_entity(self,entity):
        if isinstance(entity,self.entity_type):
            self.entity_list[entity.name] = entity
        else:
            raise TypeError(f"The entity {entity.name} must be a {self.entity_type.__name__}.")
    def remove_entity(self,entity,enabling_warning:bool=False):
        if isinstance(entity,self.entity_type):
            del self.entity_list[entity.name]
        elif enabling_warning:
            print(f"The entity {entity.name} doesn't exist.")
class GUI(EntityManager):
    def __init__(self):
        super().__init__(entity_type=GraphicsEntity)
        self.screen = pygame.display.set_mode((SCREEN_SIZE,SCREEN_SIZE))

        pygame.display.set_caption('Turtlesim+')
        pygame.init()
        module_path = os.path.abspath(__file__)
        module_dir = os.path.dirname(module_path)
        self.image_dir = os.path.join(module_dir,'image')
        self.turtle_images  = os.listdir(os.path.join(self.image_dir,'turtle'))
        self.available_images = self.turtle_images.copy()
        # Render order (back-to-front) is built by callers via register_render_order,
        # so new entity types don't require editing GUI itself.
        self.entity_order = []
    def register_render_order(self,entity_type:Type[GraphicsEntity]):
        if entity_type not in self.entity_order:
            self.entity_order.append(entity_type)
    def update(self,mouse_callback:Callable):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            # Check if the mouse button is released
            if event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1:
                    # Call your callback function with the mouse position as argument
                    pos = pygame.mouse.get_pos()
                    mouse_callback(pos)
        self.render()

    def render(self):
        self.screen.fill((0, 0, 0))  # fill the screen with white
        for entity_type in self.entity_order:
            for entity in self.entity_list.values():
               if isinstance(entity,entity_type):     
                    entity.render(self.screen)
        pygame.display.flip()
class Engine(EntityManager):
    def __init__(self,time_step:float=0.01):
        super().__init__(entity_type=PhysicsEntity)
        self.time_step = time_step
    def step(self,every_entities):
        for name,entity in self.entity_list.items():
            entity.update(self.time_step, every_entities)
class Simulator(EntityManager):
    def __init__(self,time_step:float=0.01,mouse_callback:Callable=lambda x: print('mouse is clicked')):
        super().__init__(entity_type=Entity)
        self.time_step = time_step
        self.engine = Engine(time_step=time_step)
        self.gui = GUI()
        self.mouse_callback = mouse_callback
    def add_entity(self, entity):
        if isinstance(entity,GraphicsEntity):
            self.gui.add_entity(entity=entity)
        if isinstance(entity,PhysicsEntity):
            self.engine.add_entity(entity=entity)
        super().add_entity(entity=entity)
    def remove_entity(self, entity:Entity, enabling_warning: bool = False):
        if isinstance(entity,GraphicsEntity):
            self.gui.remove_entity(entity=entity,enabling_warning=enabling_warning)
        elif isinstance(entity,PhysicsEntity):
            self.engine.remove_entity(entity=entity,enabling_warning=enabling_warning)
        super().remove_entity(entity, enabling_warning)
    
    def step(self):
        self.engine.step(every_entities=self.entity_list)
        self.gui.update(self.mouse_callback)
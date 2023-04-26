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
        screen.blit(img,(self.pose[0]/5.44*250-size,500-self.pose[1]/5.44*250-size))
        
class Turtle(PhysicsEntity,GraphicsEntity):
    def __init__(self, name: str,init_pose:List[float]=[0.0,0.0,0.0]):
        PhysicsEntity.__init__(self,name)
        self.pose = init_pose
        self.state = init_pose
    def set_pose(self, pose: List[float]):
        super().set_pose(pose=pose)
        self.state = pose
    def set_state(self, state:List[float]):
        super().set_state(state=state)
        self.set_pose(state)
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
            V = np.array([[q_dtheta[1],q_dtheta[0]-1],[1-q_dtheta[0],q_dtheta[1]]])
        R_new = np.array([q_new,[-q_new[1],q_new[0]]]).T
        dp = R_new@V@np.array([cmd_vel[0],0])
        p_new = np.array(self.pose[0:2])+dp

        p_new = [
            self.pose[0]+dt*cmd_vel[0]*math.cos(self.pose[2]),
            self.pose[1]+dt*cmd_vel[0]*math.sin(self.pose[2]),
            self.pose[2]+dt*cmd_vel[1]
        ]
        for i in range(2):
            if p_new[i]<=0:
                p_new[i] = 0.0
            if p_new[i]>=10.88:
                p_new[i] = 10.88
        self.pose = [p_new[0],p_new[1],theta_new]
        
        self.state = self.pose
    def render(self,screen):
        img = pygame.transform.rotate(self.graphics, self.pose[2]*180/math.pi-90)
        
        size = np.min([math.floor(img.get_size()[0]/2),math.floor(img.get_size()[1]/2)])  
        screen.blit(img,(self.pose[0]/5.44*250-size,500-self.pose[1]/5.44*250-size))
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
    def measure(self,pose:List[float],turtle,entity_list:List[Entity]):
        scanner_data_array = []
        entity_array = []
        for entity in entity_list.values():
            if not entity is turtle: 
            
            #if any([issubclass(type(entity), detection_type) for detection_type in self.detection_types]):
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
                if distance <= self.radius and abs(angle) <= self.range/2:
                    # create a ScannerData object and add it to the list of detected entities
                    scanner_data_array.append(ScannerData(type=type(entity),angle=angle,distance=distance))
                    entity_array.append(entity)
        return scanner_data_array,entity_array
    def render(self,screen,pose):
        vertices = [(pose[0]/5.44*250,500-pose[1]/5.44*250)]
        for i in range(int(math.degrees(-self.range / 2)), int(math.degrees(self.range / 2)) + 1):
            rads = math.radians(i)+pose[2]
            vertices.append(((pose[0] + self.radius * math.cos(rads))/5.44*250,500-(pose[1] + self.radius * math.sin(rads))/5.44*250))
        vertices.append((pose[0]/5.44*250,500-pose[1]/5.44*250))
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
    def __init__(self, turtle:Turtle):
        PhysicsEntity.__init__(self,name=turtle.name)
        self.turtle = turtle
        self.eat_range = Scanner(radius=2.0,range=math.pi/3,color=(0,255,0,127))
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
class TurtleScannerInterface(PhysicsEntity,GraphicsEntity):
    def __init__(self, turtle:Turtle):
        PhysicsEntity.__init__(self,name=turtle.name)
        self.turtle = turtle
        self.scanner = Scanner()
        self.scanner.add_detection_type(Turtle)
        self.scanner.add_detection_type(Pizza)
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
        self.screen = pygame.display.set_mode((500,500))
        
        pygame.display.set_caption('Turtlesim+')
        pygame.init()
        module_path = os.path.abspath(__file__)
        module_dir = os.path.dirname(module_path)
        self.image_dir = os.path.join(module_dir,'image')
        self.turtle_images  = os.listdir(os.path.join(self.image_dir,'turtle'))
        self.available_images = self.turtle_images.copy()
        self.entity_order = [Pizza,TurtleScannerInterface]
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
#!/usr/bin/env python3

import os
import sys
import time
import math
import argparse

#plot
import matplotlib.pyplot as plt
import networkx as nx
import matplotlib.animation as animation
import matplotlib

""" 
#######################################################
# Visualizes the FSM using NetworkX. 
# All edges and nodes have to be manually added to the graph.

# input: none
# output: none

# author: Mark Lee (MoonRobotics@cmu.edu)
# version: 1.0 (05/2023)
#######################################################
""" 


class FSM_visualizer:
    def __init__(self):
        self.G = nx.DiGraph()
        self.pos = {}
        self.fig = None
        self.ax = None

    def shutdown(self):
        plt.close("all")

    def create_graph(self):
        '''
        Create a graph for the FSM
        '''
        
        try:
            self.G.add_edge('WAIT', 'STOW')
            self.G.add_edge('STOW', 'GO2_PLANE')
            self.G.add_edge('GO2_PLANE','GO2_CAM_POSE')
            self.G.add_edge('GO2_CAM_POSE', 'REQ_DETECT')
            self.G.add_edge('REQ_DETECT', 'GO2_CORN_XY')
            self.G.add_edge('REQ_DETECT', 'GO2_SEARCH')
            self.G.add_edge('GO2_CORN_XY', 'GO2_CORN_Z')
            self.G.add_edge('GO2_CORN_Z', 'GO2_CORN_XY_NEW')
            self.G.add_edge('GO2_CORN_XY_NEW', 'INSERT_SENSOR')
            self.G.add_edge('INSERT_SENSOR', 'DEPLOY_BOX')
            self.G.add_edge('DEPLOY_BOX', 'WAIT')


            

            self.pos = {'WAIT': (2, 2), 'STOW': (0, 0), 'GO2_PLANE': (0, 1), 'GO2_CAM_POSE': (0, 2), 'REQ_DETECT': (0, 3),
                        'GO2_SEARCH': (0.5, 3), 'GO2_CORN_XY': (0, 4), 'GO2_CORN_Z': (0.5, 4), 'GO2_CORN_XY_NEW': (1, 4),
                        'INSERT_SENSOR': (2, 4), 
                        'DEPLOY_BOX': (2, 3)}

            if self.fig is None or self.ax is None:
                self.fig, self.ax = plt.subplots(figsize=(10, 6))  # Adjust the figure size according to your preference

            plt.axis('off')
            plt.ion() #non-blocking
            plt.show()

        except KeyboardInterrupt:
            print(f"shutting down plotter")
            plt.close("all")


    def highlight_all_nodes(self):
        '''
        Sequence through all nodes in the graph and highlight them
        '''
        
        def update_colors_labels(frame):
            colors = []
            labels = {}
            for n in self.G.nodes:
                if n == frame:
                    colors.append('green')
                else:
                    colors.append('white')
                labels[n] = n
            nx.draw_networkx_nodes(self.G, self.pos, node_color=colors, ax=self.ax, node_size=2000, edgecolors='black')
            nx.draw_networkx_labels(self.G, self.pos, labels=labels, font_color='black', ax=self.ax)
            nx.draw_networkx_edges(self.G, self.pos, ax=self.ax, arrows=True, arrowstyle='->')

        frames = list(self.G.nodes)
        ani = animation.FuncAnimation(self.fig, update_colors_labels, frames=frames, repeat=False)

        plt.draw()
        plt.pause(1)
        

    def highlight_only_input_node(self, node):
        '''
        Highlight only the input node
        '''

        def update_colors_labels(frame):
            colors = []
            labels = {}
            for n in self.G.nodes:
                if n == node:
                    colors.append('green')  # Highlight only the input node in green
                else:
                    colors.append('white')  # Set all other nodes to white
                labels[n] = n
            nx.draw_networkx_nodes(self.G, self.pos, node_color=colors, ax=self.ax, node_size=2000, edgecolors='black')
            nx.draw_networkx_labels(self.G, self.pos, labels=labels, font_color='black', ax=self.ax)
            nx.draw_networkx_edges(self.G, self.pos, ax=self.ax, arrows=True, arrowstyle='->')

        try:

            ani = animation.FuncAnimation(self.fig, update_colors_labels, frames=[0], repeat=False)

            plt.draw()
            plt.pause(1)

        except KeyboardInterrupt:
            print(f"shutting down plotter")
            plt.close("all")

        

# testing for plot code
if __name__ == "__main__":
    print(" ================ starting plot script ============ ")

    #for animation
    matplotlib.use('TkAgg')

    fsm = FSM_visualizer()
    fsm.create_graph()

    #all nodes
    fsm.highlight_all_nodes()

    #individual nodes
    # fsm.highlight_only_input_node('GO2_CAM_POSE')
    # time.sleep(3)
    # fsm.highlight_only_input_node('STOW')
    # time.sleep(3)

    

    print(" ================ ending plot script ============ ")



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

# testing for plot code
if __name__ == "__main__":
    print(" ================ starting script ============ ")

    #for animation
    matplotlib.use('TkAgg')

    
    G = nx.DiGraph()
    G.add_edge('STOW', 'GO2_PLANE')
    G.add_edge('GO2_PLANE','GO2_CAM_POSE' )
    G.add_edge('GO2_CAM_POSE', 'REQ_DETECT')
    G.add_edge('REQ_DETECT', 'GO2_PLANE')
    G.add_edge('REQ_DETECT', 'GO2_SEARCH')
    G.add_edge('GO2_PLANE', 'GO2_CORN')
    G.add_edge('GO2_CORN', 'INSERT')
    G.add_edge('INSERT', 'RETURN2_CORN')
    G.add_edge('RETURN2_CORN', 'RETURN2_PLANE')
    G.add_edge('RETURN2_PLANE', 'DONE')


    # explicitly set positions
    pos = {'STOW': (0, 0), 'GO2_PLANE': (0, 1), 'GO2_CAM_POSE': (0, 2), 'REQ_DETECT': (0, 3), 'GO2_SEARCH': (0.5, 3),'GO2_CORN': (0, 4), 'INSERT': (1, 4), 'RETURN2_CORN': (1, 3), 'RETURN2_PLANE': (1, 2),  'DONE': (1, 1) }

    # TODO: 
    # iterate through graph and show each node as a different color
    fig, ax = plt.subplots(figsize=(10, 6))  # Adjust the figure size according to your preference

    
    # Function to update node colors
    def update_colors_labels(frame):
        colors = []
        labels = {}
        for node in G.nodes:
            if node == frame:
                colors.append('green')
            else:
                colors.append('white')
            labels[node] = node
        nx.draw_networkx_nodes(G, pos, node_color=colors, ax=ax, node_size=2000, edgecolors='black')
        nx.draw_networkx_labels(G, pos, labels=labels, font_color='black', ax=ax)
        nx.draw_networkx_edges(G, pos, ax=ax, arrows=True, arrowstyle='->')

    frames = list(G.nodes)
    ani = animation.FuncAnimation(fig, update_colors_labels, frames=frames, repeat=False)

    plt.axis('off')
    plt.show()


   
    print(" ================ ending script ============ ")

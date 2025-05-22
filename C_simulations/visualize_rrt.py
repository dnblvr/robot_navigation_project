
import json, os
import matplotlib.pyplot as plt
import matplotlib.patches as patches


def load_json(filename):
    with open(filename, 'r') as f:
        return json.load(f)
    

def plot_from_json(data):
    fig, ax = plt.subplots(figsize=(10, 10))
    
    # Plot obstacles
    for obs in data['obstacles']:
        rect = patches.Rectangle(
            (obs['x'], obs['y']),
             obs['width'],
             obs['height'],
             linewidth=1, edgecolor='r', facecolor='r'
        )
        ax.add_patch(rect)
    
    # Plot optimal path
    if len(data['nodes']) > 0:
        # Find path from goal back to start
        path = []
        current_node = data['nodes'][-1]  # Last added node (goal-reaching node)
        
        # Trace back through parents
        while True:
            path.append((current_node['x'], current_node['y']))

            # Find parent edge
            parent_found = False
            for edge in data['edges']:

                if (abs(edge['x2'] - current_node['x']) < 1e-6 and \
                    abs(edge['y2'] - current_node['y']) < 1e-6       ):
                    current_node = next(n for n in data['nodes'] 
                                      if abs(n['x'] - edge['x1']) < 1e-6 and 
                                         abs(n['y'] - edge['y1']) < 1e-6)
                    parent_found = True
                    break

            if not parent_found or (abs(current_node['x']) < 1e-6 and 
                                    abs(current_node['y']) < 1e-6     ):
                break

        # Reverse to get start-to-goal order and plot
        path.reverse()
        x_vals = [p[0] for p in path]
        y_vals = [p[1] for p in path]
        ax.plot(x_vals, y_vals, 'g-', linewidth=3, label='Optimal Path', zorder=3)
    

    
    # Plot all nodes
    nodes = data['nodes']
    ax.plot([n['x'] for n in nodes], [n['y'] for n in nodes], 'o', markersize=2, color='blue', alpha=0.5)
    
    # Plot all edges
    for edge in data['edges']:
        ax.plot([edge['x1'], edge['x2']], [edge['y1'], edge['y2']], 
                color='gray', linewidth=0.5, alpha=0.3)
        

    # Plot goal region
    goal = data['goal']
    circle = plt.Circle(
        (goal['x'], goal['y']),
        goal['radius'],
        color='green', fill=False, linewidth=2
    )
    ax.add_patch(circle)
    
    # Plot start node
    ax.plot(data['nodes'][0]['x'], data['nodes'][0]['y'], 'gs', 
            markersize=10, label='Start', zorder=4)
    
    # Plotting properties
    ax.set_aspect('equal')
    plt.xlim(0, 100)
    plt.ylim(0, 100)
    plt.title('RRT* Path Planning Visualization')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.show()

  
os.chdir('/Users/gianfajardo/Documents/Sync\'d Documents/ECE 595R & L - Robotic Systems with Lab/Final Project/code/Robot_Navigator_main/robot_navigation_project/vscode_RRT_star')

data = load_json(r"rrt_tree.json")
plot_from_json(data)

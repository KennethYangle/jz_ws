from numpy import zeros

def generate_map_64000(level_id):
    obs_value = 100
    grid_map = zeros((160, 400))
    if level_id == 1:
        horizontal_obstacles = [(0, (range(40, 357), range(375, 400))), (39, (range(160, 220), range(350, 400))), (159, (range(400), ))]
        for o in horizontal_obstacles:
            x, yrt = o
            for yr in yrt:
                for y in yr:
                    grid_map[x][y] = obs_value
        vertical_obstacles = [((range(160), ), 0), ((range(160), ), 39), ((range(40, 160), ), 219), ((range(160), ), 399)]
        for o in vertical_obstacles:
            xrt, y = o
            for xr in xrt:
                for x in xr:
                    grid_map[x][y] = obs_value
    elif level_id == 2:
        horizontal_obstacles = [(0, (range(40, 208), range(224, 400))), (119, (range(40, 324), range(360, 400))), (159, (range(400), ))]
        for o in horizontal_obstacles:
            x, yrt = o
            for yr in yrt:
                for y in yr:
                    grid_map[x][y] = obs_value
        vertical_obstacles = [((range(160), ), 0), ((range(120), ), 39), ((range(60), ), 159), ((range(160), ), 399)]
        for o in vertical_obstacles:
            xrt, y = o
            for xr in xrt:
                for x in xr:
                    grid_map[x][y] = obs_value
        rectangles = [(range(40, 60), range(80, 120))]
        for o in rectangles:
            xr, yr = o
            for x in xr:
                for y in yr:
                    grid_map[x][y] = obs_value
    else:
        print('LEVEL ERROR')
    return grid_map

def generate_map(level_id):  # 16000
    obs_value = 100
    grid_map = zeros((80, 200))
    if level_id == 1:
        horizontal_obstacles = [(0, (range(20, 179), range(187, 200))), (19, (range(80, 110), range(175, 200))), (79, (range(200), ))]
        for o in horizontal_obstacles:
            x, yrt = o
            for yr in yrt:
                for y in yr:
                    grid_map[x][y] = obs_value
        vertical_obstacles = [((range(80), ), 0), ((range(80), ), 19), ((range(20, 80), ), 109), ((range(80), ), 199)]
        for o in vertical_obstacles:
            xrt, y = o
            for xr in xrt:
                for x in xr:
                    grid_map[x][y] = obs_value
    elif level_id == 2:
        horizontal_obstacles = [(0, (range(20, 104), range(112, 200))), (59, (range(20, 150), range(160, 200))), (79, (range(200), ))]
        for o in horizontal_obstacles:
            x, yrt = o
            for yr in yrt:
                for y in yr:
                    grid_map[x][y] = obs_value
        vertical_obstacles = [((range(80), ), 0), ((range(60), ), 19), ((range(30), ), 79), ((range(80), ), 199)]
        for o in vertical_obstacles:
            xrt, y = o
            for xr in xrt:
                for x in xr:
                    grid_map[x][y] = obs_value
        rectangles = [(range(23, 34), range(39, 60))]
        for o in rectangles:
            xr, yr = o
            for x in xr:
                for y in yr:
                    grid_map[x][y] = obs_value
    else:
        print('LEVEL ERROR')
    return grid_map

def generate_targets(level_id):
    target_pos = []
    if level_id == 1:
        target_pos += [(20, -12), (33, -5.5)]
    elif level_id == 2:
        target_pos += [(39, -15), (15, -18), (28, -5.5)]
    else:
        print('LEVEL ERROR')
    
    ##############correcting
    tar_pos = []
    for t in target_pos:
        tar_pos.append((t[0] - 13, t[1] + 18))
    ########################

    return tar_pos

def generate_uav_init(level_id, uav_id=-1):
    uav_init = []
    if level_id == 1:
        uav_init += [[13, -18], [23, -18]]
    elif level_id == 2:
        uav_init += [[26.5, -18], [24.5, -18]]
    else:
        print('LEVEL ERROR')
    
    ##############correcting
    uav_pos = []
    for t in uav_init:
        uav_pos.append((t[0] - 13, t[1] + 18))
    ########################

    if uav_id >= 0:
        return uav_pos[uav_id]
    else:
        return uav_pos        


if __name__ == '__main__':
    import matplotlib.pyplot as plt
    
    lid = 2
    gmap = generate_map(lid)
    plt.matshow(gmap)
    plt.show()


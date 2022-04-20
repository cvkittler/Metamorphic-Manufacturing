# The final 2d array of points for checking
final_array = []

# Open the file and read the lines into an array, give it path of file
pc_file = open('1.pcd', 'r')
lines = pc_file.readlines()

# For each line, split it, remove the last element, cast each remaining element to float and add to final array
for line in lines[11:20]:
    temp = line.split()
    temp.pop()
    temp = list(map(float, temp))
    final_array.append(temp)

# Check if output is correct
# for stuff in final_array:
#     print(stuff)

# Cube with 27 points centered around 0,0,0
cube1 = [[-1, -1, -1], [0, -1, -1], [1, -1, -1], [-1, 0, -1], [0, 0, -1], [1, 0, -1], [-1, 1, -1], [0, 1, -1], [1, 1, -1], # Bottom layer
        [-1, -1, 0], [0, -1, 0], [1, -1, 0], [-1, 0, 0], [0, 0, 0], [1, 0, 0], [-1, 1, 0], [0, 1, 0], [1, 1, 0], # Middle layer
        [-1, -1, 1], [0, -1, 1], [1, -1, 1], [-1, 0, 1], [0, 0, 1], [1, 0, 1], [-1, 1, 1], [0, 1, 1], [1, 1, 1]] # Top layer

# defect on bottom
cube2 = [[-1, -1, -1], [0, -1, -2], [1, -1, -1], [-1, 0, -1], [0, 0, -1], [1, 0, -1], [-1, 1, -1], [0, 1, -1], [1, 1, -1], # Bottom layer
         [-1, -1, 0], [0, -1, 0], [1, -1, 0], [-1, 0, 0], [0, 0, 0], [1, 0, 0], [-1, 1, 0], [0, 1, 0], [1, 1, 0], # Middle layer
         [-1, -1, 1], [0, -1, 1], [1, -1, 1], [-1, 0, 1], [0, 0, 1], [1, 0, 1], [-1, 1, 1], [0, 1, 1], [1, 1, 1]] # Top layer

# defect on bottom and top
cube3 = [[-1, -1, -1], [0, -1, -2], [1, -1, -1], [-1, 0, -1], [0, 0, -1], [1, 0, -1], [-1, 1, -1], [0, 1, -1], [1, 1, -1], # Bottom layer
         [-1, -1, 0], [0, -1, 0], [1, -1, 0], [-1, 0, 0], [0, 0, 0], [1, 0, 0], [-1, 1, 0], [0, 1, 0], [1, 1, 0], # Middle layer
         [-1, -1, 1.1], [0, -1, 1], [1, -1, 1], [-1, 0, 1], [0, 0, 1], [1, 0, 1], [-1, 1, 1], [0, 1, 1], [1, 1, 1]] # Top layer

def check_faces(object):

    wrong_point_list = []
    anything_wrong = False

    # Checks a cube centered at 0,0 that extends out 1 unit in each direction
    # Each 'if' checks a side/range for an object, this is hardcoded currently
    for r in range(27):
        if(object[r][2] < -1): # check bottom
            wrong_point_list.append(object[r])
            anything_wrong = True
        if(object[r][2] > 1): # check top
            wrong_point_list.append(object[r])
            anything_wrong = True
        if(object[r][0] < -1): # check left
            wrong_point_list.append(object[r])
            anything_wrong = True
        if(object[r][0] > 1): # check right
            wrong_point_list.append(object[r])
            anything_wrong = True
        if(object[r][1] < -1): # check front
            wrong_point_list.append(object[r])
            anything_wrong = True
        if(object[r][1] > 1): # check back
            wrong_point_list.append(object[r])
            anything_wrong = True

    if not anything_wrong:
        print("No points out of bounds")
    else:
        print("Points that are out of bounds")
        for p in wrong_point_list:
            print(p)

# Input for check_faces() is the converted point cloud that you would like to check
# Currently its using the premade cube3 array
check_faces(cube3)

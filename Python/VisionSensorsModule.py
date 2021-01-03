#!/usr/bin/env python3

""" An OOP design for simulating vision sensors (cameras and lidars) from scratch.
    The design includes Body, Camera and Lidar hierarchical classes and some auxiliary helpers.
    The pointcloud from a velodyne VLP16 lidar looking down at a car is synthesized as a demo in the main function.
    Ths demo shows how to use the module, i.e. the Body and Lidar classes and the Aux and Pointcloud facilities.

    Arash Mohtat
    Dec 2020
"""


import os
import time
import numpy as np
from scipy.linalg import orth
import pandas as pd
from PIL import Image, ImageDraw
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.pyplot as plt

def getFullFilePath(subdir, filename):
    dir_path = os.path.join(os.path.dirname(__file__), subdir)
    return os.path.join(dir_path, filename)

def readFile(file_name, to_type):
        matrix = []
        with open(file_name, 'r') as f:
            lines = f.readlines()
            for line in lines:
                coords = line.strip().split(',')
                matrix.append([to_type(coords[0]), to_type(coords[1]), to_type(coords[2])])
            f.close()
        return np.array(matrix)


class Body:
    def __init__(self, tag, transform=np.eye(4), reference=None):
        self.points = np.array([]) # will be a 4*N array (last row of 1's for homogenous coords)
        self.triangles = np.array([]) # will be an n*3 array
        self.tag = tag
        self.transform = transform.copy()
        #self.reference = reference

    def copy(self):
        copied = Body(self.tag + '_copy', transform=self.transform)
        copied.points = self.points.copy()
        copied.triangles = self.triangles.copy()
        return copied
        
    def assignGeometry(self, vertices, triangles, folder_name=None):
        if folder_name is None:
            pts, trngls = vertices, triangles
        else:
            pts = readFile(getFullFilePath(folder_name, vertices), lambda x: float(x))
            trngls = readFile(getFullFilePath(folder_name, triangles), lambda x: int(x))
            trngls = trngls - trngls.min()  # in case triangle indices don't start from 0
        
        self.points = np.vstack((pts.transpose(), np.ones(pts.shape[0]))) # note: pts is N*3, self.points is 4*N
        self.triangles = trngls
        
    def move(self, delta_xyz):
        self.transform[0:3,3] = self.transform[0:3,3] + delta_xyz

    def rotate(self, axis, angle_deg):
        # applies a single canonical rotation to the Body's transform property

        angle_rad = angle_deg * np.pi / 180.0
        c, s = np.cos(angle_rad), np.sin(angle_rad)
        rotation = np.eye(3)
        axis_slices = {'z': slice(0,2), 'x': slice(1,3), 'y': slice(0,3,2)}
        rotation[axis_slices[axis], axis_slices[axis]] = np.array([[c, -s],[s, c]])

        self.transform[0:3, 0:3] = rotation @ self.transform[0:3, 0:3] # pre-multiply or post-multiply?

    def append(self, other):
        self.triangles = np.vstack((self.triangles, 1 + np.max(self.triangles) + other.triangles))
        T = np.linalg.inv(self.transform) @ other.transform
        self.points = np.hstack((self.points, T @ other.points))
 
    def getPolygons(self, view_transform=np.eye(4)):
        # view_transform is needed if polygons are desired in anything other than world coordinates
        # e.g. if body polygons is desired from a vision sensor view, use inverse of sensor transform 

        polygon_list = []
        points = (view_transform @ self.transform) @ self.points
        for trngl in self.triangles:
            polygon_list.append(np.row_stack([points[0:3,trngl[vertex]] for vertex in range(3)]))
        return polygon_list

class Camera(Body):
    def __init__(self, tag, pixel_specs, transform=np.eye(4), reference=None):
        super().__init__(tag, transform, reference)
        self.width = pixel_specs[0]
        self.height = pixel_specs[1]
        self.focal_length = pixel_specs[2]

    def transformCoords(self, target):
        # transform target vertices to camera coordinates
        A = np.linalg.inv(self.transform) @ target.transform # there are more efficient ways of inverting (not too critical)
        vertices_hom = A @ target.points # transformed vertices homogenous coords
        return vertices_hom[0:3, :] # strip away 4th row of 1's

    def project2pixels(self, target, return_distance=False):
        """ perspective projection of target vertices on image plane"""
        
        # target vertices in camera coordinates
        vertices = self.transformCoords(target) 
        
        #calculate 2D coordinates from 3D points
        x, y, z = vertices[0, :], vertices[1, :], vertices[2, :]
        u = x * self.focal_length / y + self.width/2
        v = self.height/2 - z * self.focal_length / y

        if return_distance:
            return np.column_stack((u, v)), np.sqrt(x**2 + y**2 + z**2) # (spherical) distance of vertices to pinhole
        else:
            return np.column_stack((u, v)), y # note: y is depth (projected distance of vertices to pinhole, perpendicular to image plane)

    def renderView(self, target, background=(240,248,255), fill_color='red', show=True):
        
        # project vertices and calculate triangular mesh depths
        projected_points, depth = self.project2pixels(target)  
        avgZ = -(depth[target.triangles[:,0]] + depth[target.triangles[:,1]] + depth[target.triangles[:,2]]) / 3

        # prep image
        im = Image.new("RGB", (self.width, self.height), color=background)
        draw = ImageDraw.Draw(im)
        draw.rectangle([0,0,self.width-1,self.height-1], fill=None, outline='black')

        #sort and draw triangles from furthest back to closest
        idx_sort = np.argsort(avgZ)
        faces = target.triangles[idx_sort,:]
        for row in faces:
            draw.polygon(list(projected_points[row,:].flatten().astype(int)), fill =fill_color, outline ='black')
        if show:
            im.show()

        return im, draw

    def renderDepth(self, target, draw=False):

        def bufferDepth(arg1, arg2):
            # TODO: better to use OpenGL: otherwise need to create traingulation pixel-level interpolation
            # and it won't be efficient if not done properly and without GPU!
            pass
        
        # project vertices and calculate triangular mesh depths
        projected_points, depth = self.project2pixels(target)  
        avgZ = -(depth[target.triangles[:,0]] + depth[target.triangles[:,1]] + depth[target.triangles[:,2]]) / 3

        #sort triangles and overwrite interpolated pixel depths from furthest back to closest triangle
        idx_sort = np.argsort(avgZ)
        faces = target.triangles[idx_sort,:]
        for row in faces:
            bufferDepth(projected_points[row,:], depth[row])


class Lidar(Camera):
    def __init__(self, tag, hFOV=120.0, hRes=120, vFOV=30.0, vRes=16, transform=np.eye(4), reference=None):
        
        # initialize equivalent parent pinhole camera
        hFOV_rad = np.radians(hFOV)
        vFOV_rad = np.radians(vFOV)
        focal_length = max(1000, 1+int(hRes/(2.0 * np.tan(hFOV_rad/2.0))))  
        width = int(2.0 * focal_length * np.tan(hFOV_rad/2.0))
        height = int(2.0 * focal_length * np.tan(vFOV_rad/2.0) / np.cos(hFOV_rad/2.0)) 
        pixel_specs = (width, height, focal_length)   
        super().__init__(tag, pixel_specs, transform, reference)
        self.FOV_deg = (hFOV, vFOV)
        self.resolution = (hRes, vRes)

        # prepare scanning grids and initialize empty measurement grids
        azimuth = np.linspace(-hFOV_rad/2.0, hFOV_rad/2.0, hRes)
        elevation = np.linspace(vFOV_rad/2.0, -vFOV_rad/2.0, vRes)
        [AZ_grid, EL_grid] = np.meshgrid(azimuth, elevation) # left to right, bottom-down on image plane (AZ cols, EL rows)
        u_grid = width/2.0 + focal_length*np.tan(AZ_grid)      # u, v will be converted to integers only for plotting pixels
        v_grid = height/2.0 - focal_length*np.tan(EL_grid)/np.cos(AZ_grid)
        self.grid = {'AZ': AZ_grid, 'EL': EL_grid, 'u': u_grid, 'v': v_grid, 
                     'rho': np.ones_like(AZ_grid)*np.inf, 'x': np.zeros_like(AZ_grid),     
                     'y': np.zeros_like(AZ_grid), 'z': np.zeros_like(AZ_grid)}

    def renderView(self, target, background=(240,248,255), fill_color='red', show=True):
        im, draw = super().renderView(target, background, fill_color, show=False) # call camera's method
        for channel in range(self.resolution[1]):
            uv = np.column_stack((self.grid['u'][channel,:], self.grid['v'][channel,:]))
            draw.line(list(uv.flatten().astype(int)), fill='blue', width=2)
        if show:
            im.show()
        return im, draw

    def transformCoords(self, target, coords = 'cart'):    
        vertices_cart = super().transformCoords(target) # xyz rows (Cartesian coords)
        if coords == 'cart':
            return vertices_cart
        vertices_sphr = np.empty_like(vertices_cart)    # rho-AZ-EL rows (Spherical coords)
        vertices_sphr[0, :] = np.linalg.norm(vertices_cart, ord=None, axis=0)
        vertices_sphr[1, :] = np.arctan2(vertices_cart[0, :], vertices_cart[1, :])
        vertices_sphr[2, :] = np.arcsin(vertices_cart[2, :]/vertices_sphr[0, :])
        if coords == 'sphr':
            return vertices_sphr
        elif coords == 'both':
            return vertices_cart, vertices_sphr
        else:
            raise ValueError('Unknown coords')
    
    @staticmethod
    def sliceLen(s):
        return s.stop - s.start # assuming s.step = None, i.e. 1
    
    def castRays(self, trngl_xyz_rows, n, AZ_slice, EL_slice):
        """ casts spherical rays on a 3D triangle, and updates distance (rho) state of self (lidar object)
            trngl_xyz: 3-by-3 matrix with x,y,z rows (each column represents a vertex), 
            n: unit-normal vector of plane defined by triangle as an np array of shape (3,)
            AZ_slice, EL_slice: enable slicing the relevant part of the grid, i.e. self.grid['rho'][EL_slice, AZ_slice]
        """

        # construct flattened sine and cosine of AZ and EL of rays
        AZ_flat = self.grid['AZ'][EL_slice, AZ_slice].flatten()
        EL_flat = self.grid['EL'][EL_slice, AZ_slice].flatten()
        ca = np.cos(np.pi/2 - AZ_flat)
        sa = np.sin(np.pi/2 - AZ_flat)
        ce = np.cos(EL_flat)
        se = np.sin(EL_flat) 

        # calculate the intersection of rays (rho) with the triangle plane
        d = np.dot(trngl_xyz_rows[:,0], n)
        rho = d/(n[0]*ce*ca+n[1]*ce*sa+n[2]*se) # d -> d-n[2]*ver_corr if rays have a vertical correction
        rho[rho<0] = np.inf # excludes the intersection from backward extension of rays
        
        pts_xyz_rows = np.row_stack([rho*ce*ca, rho*ce*sa, rho*se]) # rho*se -> rho*se+ver_corr if needed
        cond = Aux.areInTriangle(pts_xyz_rows, trngl_xyz_rows, n) # check if points lie in the triangle actually
        rho[~cond] = np.inf

        # update the grid's rho (broadcast into 2d shape and min-accumulate)
        self.grid['rho'][EL_slice, AZ_slice] = np.minimum(self.grid['rho'][EL_slice, AZ_slice],
                                                          rho.reshape((Lidar.sliceLen(EL_slice), Lidar.sliceLen(AZ_slice))))
 
    def findBoundingRays(self, trngl_RhoAzEl):
        
        # slices for entire grid (for testing)
        # EL_slice = slice(0, self.grid['rho'].shape[0]) # number of rows
        # AZ_slice = slice(0, self.grid['rho'].shape[1]) # number of cols

        # find relevant AZ slice (azimuths are listed in ascending order in grid rows)
        AZ_minmax = (np.min(trngl_RhoAzEl[1,:]), np.max(trngl_RhoAzEl[1,:]))
        AZ_vals = self.grid['AZ'][0, :]
        AZ_idx = np.searchsorted(AZ_vals, AZ_minmax)
        AZ_slice = slice(AZ_idx[0], AZ_idx[1])

        # find EL slice (elevations are listed in descending order in grid columns)
        EL_maxmin = (np.max(trngl_RhoAzEl[2,:]), np.min(trngl_RhoAzEl[2,:]))
        EL_vals = np.flip(self.grid['EL'][:, 0])
        EL_idx = np.searchsorted(EL_vals, EL_maxmin)
        EL_slice = slice(len(EL_vals)-EL_idx[0], len(EL_vals)-EL_idx[1])
        
        return EL_slice, AZ_slice       
    
    def scan(self, target, reset_measurements=True):
           
        vertices_cart, vertices_sphr = self.transformCoords(target, coords='both')  # target vertices in lidar cart and sphr coords
        if reset_measurements:
            self.grid['rho'] = np.ones_like(self.grid['AZ'])*np.inf
        
        # loop through triangles (each row defines 3 vertex indices defining a trngl)
        for row in target.triangles:
            trngl_xyz = vertices_cart[:, row] # 3-by-3 matrix with x,y,z rows
            valid, n = Aux.isValidTriangle(trngl_xyz)
            if not valid:
                continue # if triangle is ill-defined, then skip this iteration
            EL_slice, AZ_slice = self.findBoundingRays(vertices_sphr[:, row]) # perhaps this can be done before and triangles can be ordered based on average vertices row from close to far (and we can break out of this loop when grid['rho'] contains no inf!)          
            if Lidar.sliceLen(EL_slice) == 0 or Lidar.sliceLen(AZ_slice) == 0:
                continue # if slice is empty, then skip this iteration
            self.castRays(trngl_xyz, n, AZ_slice, EL_slice) # internally accumulates on self.grid['rho']
        
        # spherical transform when all rho's are updated (add channel vertical corrections if needed)
        self.grid['x'] = self.grid['rho']*np.cos(self.grid['EL'])*np.sin(self.grid['AZ'])
        self.grid['y'] = self.grid['rho']*np.cos(self.grid['EL'])*np.cos(self.grid['AZ'])
        self.grid['z'] = self.grid['rho']*np.sin(self.grid['EL'])

        return self.grid

class PointCloud: # just a namespace for now (will be made into a proper class if needed)
    
    @staticmethod
    def grid2DataFrame(grid_dict):
        return pd.DataFrame(np.column_stack([grid.flatten() for grid in grid_dict.values()]), columns=grid_dict.keys())
    
    @staticmethod
    def draw(grid_dict=None, file_name=None, ax=None, show=True):
        if grid_dict is not None:
            df = PointCloud.grid2DataFrame(grid_dict)
        elif file_name is not None:
            df = pd.read_csv(getFullFilePath('renderings', file_name))
        else:
            raise ValueError('A grid dictionary or a data file needs to be provided')

        if ax is None:
            ax = Aux.makeNewAxis()

        df_finite = df[df['rho']!=np.inf] 
        ax.scatter(df_finite['x'], df_finite['y'], df_finite['z'], marker='o', c="limegreen")
        
        plt.figure()
        plt.plot(list(df_finite.index), df_finite['rho'])
        
        
        if show:           
            plt.show()
        return ax

    @staticmethod
    def save(grid_dict, file_name, format='csv'):
        """ saves a csv table or as a pickle dump """
        df = PointCloud.grid2DataFrame(grid_dict)
        if format == 'csv':  
            df.to_csv(getFullFilePath('renderings', file_name + '.csv'))
        elif format == 'xyz':
            df = df.replace([np.inf, -np.inf], np.nan).dropna().reset_index()
            df_xyz = df[['x','y','z']]
            df_xyz.to_csv(getFullFilePath('renderings', file_name + '.xyz'),
                          sep=' ', index=False, header=False)

class Aux: # just a namespace for auxiliary helper functions
    @staticmethod
    def makeNewAxis():
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')
        return ax 

    @staticmethod
    def drawPolygons(polygon_list, ax=None, show=True):
        if ax is None:
            ax = Aux.makeNewAxis()
        ax.add_collection3d(Poly3DCollection(polygon_list))
        if show:           
            plt.show()
        return ax

    @staticmethod
    def isValidTriangle(trngl_xyz, tol=1e-6):
        valid = False
        n = np.cross(trngl_xyz[:,1] - trngl_xyz[:,0], trngl_xyz[:,2] - trngl_xyz[:,0])
        norm_n = np.linalg.norm(n)
        if norm_n > tol:
            valid = True
            n = n/norm_n
        return valid, n
    
    @staticmethod
    def areInTriangle(pts_xyz_rows, trngl_xyz_rows, n):
        """ determines if pts_xyz_rows (3-by-n) are in the triangle defined by vertices
        trngl_xyz_rows[:,i] for i=0,1,2 and the unit normal n. The function is vectorized
        and returns a Boolean np-array of shape (n,). """
        
        #n_col = np.expand_dims(n, axis=1)
        #P = np.eye(3,3)- n_col @ n_col.transpose() # projection matrix
        #T = orth(P) # 3-by-2 Transformation matrix (xyz coords to XY coords on triangle plane)
        v1 = trngl_xyz_rows[:, 1] - trngl_xyz_rows[:, 0] # vector from vertex A to B
        v2 = np.cross(n, v1/np.linalg.norm(v1))
        T = np.column_stack((v1, v2))
        pts_XY_rows = T.transpose() @ pts_xyz_rows # 2-by-n matrix of points in XY coords
        trngl_XY_rows = T.transpose() @ trngl_xyz_rows # 2-by-3 matrix of trngl vertices in XY coords
        A = trngl_XY_rows[:, 0:1] # XY coord of trng vertex A (keep it a 2-by-1 array)
        B = trngl_XY_rows[:, 1] - A[:, 0] # XY coords of vertex B (from A as origin)
        C = trngl_XY_rows[:, 2] - A[:, 0] # XY coords of vertex C (from A as origin)
        P = pts_XY_rows - A # XY coords of points P (from A as origin)
        area = B[0]*C[1]-C[0]*B[1] # area of triangle (nonzero as vertices are not collinear)
        
        # Barycentric coords of points P
        wA = (P[0,:]*(B[1]-C[1]) + P[1,:]*(C[0]-B[0]) + B[0]*C[1] - C[0]*B[1]) / area
        wB = (P[0,:]*C[1] - P[1,:]*C[0]) / area
        wC = (P[1,:]*B[0] - P[0,:]*B[1]) / area
        conditions = np.row_stack([wA>=0, wA<=1, wB>=0, wB<=1, wC>=0, wC<=1])
        
        return np.all(conditions, 0) # columns with all True rows refer to points P in trngl

def main():
    vehicle = Body('hatchback')
    vehicle.assignGeometry('hatchback_vertices_vehcoords.txt', 'hatchback_triangles_vehcoords.txt', folder_name='coords')
    vehicle.move(np.array([2.0, 6.0, 0]))

    ground = Body('Ground_2trngls') #, transform=np.eye(4))
    ground.assignGeometry(np.array([[-20.0,0,0],[-20.0,30,0],[20.0,0,0],[20.0,30,0]]), np.array([[0,1,2],[2,3,1]]))
    print(f'Ground transform is: {ground.transform}')
    print(f'Ground: min_y={ground.points[1,:].min()}, max_y={ground.points[1,:].max()}')

    scene = ground
    scene.append(vehicle)

    ldr = Lidar(tag='lidar', hFOV=90, hRes=900)
    ldr.move(np.array([0.0, 0.0, 3.0]))
    ldr.rotate('x', -15)
    
    ldr.renderView(scene)
    t = time.time()
    grid_dict = ldr.scan(scene)
    print(f'Elapsed time is {time.time()-t:0.1f} seconds')
    
    PointCloud.save(grid_dict, 'ptcld_vehicle_grnd_2mRight','xyz')
    PointCloud.draw(grid_dict=grid_dict, ax=None, show=True)

if __name__ == "__main__":
    main()     

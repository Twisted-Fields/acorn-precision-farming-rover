"""
Main Curver file:
Curve reconstruction from noisy data

Based on "Curve reconstruction from unorganized points",
In-Kwon Lee, Computer Aided Geometric Design 17

kieranrcampbell@gmail.com
"""

import numpy as np
import statsmodels.api as sm

from scipy.optimize import minimize

"""
NB in general first column of data will by X
and second column Y, though just convention
"""

class Curver:
	""" Class for curve reconstruction """

	def __init__(self, points=None):
		self.points = points
		self.original_points = points
		self.W = None # weight matrix
		self.N = points.shape[0]

		""" For the last transformation (M) performed,
		what theta and what point was used in order to
		reconstruct the reverse transformation """
		self.current_theta = None
		self.current_point = None



	def from_csv(self, filename):
		""" Load point cloud from filename csv """
		points = np.genfromtxt(filename, delimiter=",")
		assert points.shape[1] == 2

		self.N = points.shape[0]
		self.points = points
		self.original_points = points


	def get_points(self):
		""" Returns the original set of points """
		return self.points

	def reconstruct(self, H=10, niter=5):
		""" reconstruction routine """
		self._weight_matrix(H)

		new_pointset = np.zeros(shape=self.points.shape)

		for it in range(niter):
			for i in range(0,self.points.shape[0]):
				[xy, W, ab, new_point_index] = self.hat_transformation(i)
				[a,b,c] = self.quadratic_fit(xy, W)
				p_star = self.rev_hat_transformation(np.array([0,c]))
				new_pointset[i,:] = p_star
			self.points = new_pointset

		return self.points


	def _weight_matrix(self, H):
		R = np.zeros(shape=(self.N,self.N))
		"""
		Fill out bottom left corner of W then add to transpose
		"""

		for i in range(1, self.N):
			for j in range(0, i):
				x = self.points[i,:] - self.points[j,:]
				R[i,j] = x.dot(x)

		R = R + R.T

		w = 2 / H**3 * np.power(R,3) - 3 / H**2 * np.power(R,2) + 1
		w[R > H] = 0
		np.fill_diagonal(w, 1)

		self.W = w

	def _regression_vectors(self, point_index):
		""" Constructs Y, X and weights for the D_l
		regression
		"""
		orig_point = self.points[point_index,:]
		weights = self.W[point_index,:]
		points = self.points[weights > 0,:]

		new_point_index = int(np.where(np.all(points == orig_point,axis=1,))[0])
		weights = weights[weights > 0]

		assert points.shape[0] == len(weights), "%d rows of points isn't equal to %d length of weights" % (points.shape[0],len(weights))

		Y = points[:,1]
		X = np.column_stack([np.ones(points.shape[0]), points[:,0]])
		return [Y,X,weights, new_point_index]

	def _do_first_regression(self, Y, X, W):
		""" Performs the initial regression step
		Minimises D_l for the point specified by point point_index
		"""

		wls_model = sm.WLS(Y, X, weights = 1.0 / W)
		results = wls_model.fit()
		b, a = results.params # convention from paper
		return (a,b)

	def _rotation_from_gradient(self,m):
		""" Two dimensional rotation matrix to rotate a
		line parallel with x-axis given gradient m
		If m is negative, we want to rotate through
		| arctan(m) | degrees, while if m is positive
		we want to rotate through -|arctan(m)|. But
		sgn(arctan(m)) = sgn(m), so set
		theta = - np.arctan(m).

		"""
		theta = -np.arctan(m)
		self.current_theta = theta
		return self._rotation_from_angle(theta)

	def _rotation_from_angle(self,theta):
		# print "Making rotation matrix with angle %f rad" % theta
		r_matrix = np.array([[np.cos(theta), -np.sin(theta)],
							[np.sin(theta), np.cos(theta)]])
		return r_matrix


	def hat_transformation(self, point_index):
		""" Forward hat transformation """
		[Y,X,W,new_point_index] = self._regression_vectors(point_index)
		a,b = self._do_first_regression(Y,X,W)

		rotation_matrix = self._rotation_from_gradient(a)

		XY = np.column_stack([X[:,1],Y])
		point = self.points[point_index,:]
		self.current_point = point

		# adjust coordinates to be at origin
		XY = XY - point

		XY_rot = np.dot(rotation_matrix, XY.T).T
		return [XY_rot, W, (a,b),new_point_index]

	def rev_hat_transformation(self, P_star):
		""" Reverse hat transformation on a
		vector P_star = (0,c) -> (x,y) """

		rotation_matrix = self._rotation_from_angle(self.current_theta)
		P_star = np.dot(rotation_matrix, P_star)
		P_star = P_star + self.current_point
		return P_star




	def quadratic_fit(self, xy, W):
		""" For xy = (x,y) fit a weighted (by W)
		regression of the form
		y = ax^2 + bx + c, then return c """
		# first construct the independent matrix
		y = xy[:,1]
		x = xy[:,0]
		X = np.column_stack([np.ones(len(x)),
							x, np.power(x,2)])

		wls_model = sm.WLS(y, X, weights = 1.0/W)
		wls_model_results = wls_model.fit()
		[c,b,a] = wls_model_results.params
		return [a,b,c]



# def _q_objective_func(c, b, a, data):
# 	""" Objective function
# 	\sum_i (ax_i^2 + bx_i + c - y_i)^2 * w_i
# 	"""
# 	params = np.array([c,b,a])
# 	y, X, w = data
# 	xp = params * X
# 	s = np.power(xp.sum(1) - y, 2)
# 	return (s * w).sum()

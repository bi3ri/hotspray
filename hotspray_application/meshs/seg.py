import igl
#import networkx as nx
import meshplot as mp
from sklearn.cluster import spectral_clustering
sv, sf = igl.read_triangle_mesh("results_mesh.ply")
A = igl.adjacency_matrix(sf)
#A = nx.adjacency_matrix(sf)
labels = spectral_clustering(A)
mp.offline()
mp.plot(sv, sf, labels)
#include "cluster.h"
#include <math.h>

std::vector<point4D> hcluster(std::vector<point4D> v) {
	int i, j, ind1, ind2, elems = v.size(), max_cluster = 0;
	int inds[elems];
	float mindist = 100;
	std::vector<std::vector<float> > dists(elems, std::vector<float>(elems));
	std::vector<std::vector<point4D> > cluster_inds(elems);
	std::vector<float> mins(elems, 100.0f);
	std::vector<int> cluster(elems);
	for(i = 0; i < elems; i++) {
		cluster_inds[i].push_back(v[i]);
		cluster[i] = i;
		mindist = 100;
		for(j = i; j< elems; j++) {
			if(i == j) {
				dists[i][j] = 100;
				continue;
			}
			dists[i][j] = calc_dist(v[i], v[j]);
			dists[j][i] = dists[i][j];
			if(dists[i][j] < mins[i]) {
				inds[i] = j;
				mins[i] = dists[i][j];
			}
			if(dists[j][i] < mins[j]) {
				mins[j] = dists[j][i];
				inds[j] = i;
			}
		}
	}
	std::vector<point4D> out;
	do {
		mindist = 100;
		for(i = 0; i < elems; i++) {
			if(mins[i] < mindist) {
				mindist = mins[i];
				ind1 = i;
				ind2 = inds[i];
			}
		}
		cluster_inds[ind1].insert( cluster_inds[ind1].end(), cluster_inds[ind2].begin(), cluster_inds[ind2].end() );
		if(cluster_inds[ind1].size() > max_cluster) {
			max_cluster = cluster_inds[ind1].size();
			out = cluster_inds[ind1];
		}
		mins[ind1] = 100;
		mins[ind2] = 100;
		// Take min from matrix
		for(j = 0; j < elems; j++) {
			if(dists[ind2][j] < dists[ind1][j] && j != ind1) {
				dists[ind1][j] = dists[ind2][j];
				dists[j][ind1] = dists[ind2][j];
			}
			if(dists[ind1][j] < mins[ind1] && j != ind1 && j != ind2) {
				mins[ind1] = dists[ind1][j];
				inds[ind1] = j;
			}
			dists[ind2][j] = 100; dists[j][ind2] = 100;
			if(inds[j] == ind2) {
				inds[j] = ind1;
			}
		}
	} while(mindist < CLUSTER_THRESH);
	return out;
}

static inline float calc_dist(point4D a, point4D b) {
	float dist;
	dist += (a.point[0] - b.point[0]) * (a.point[0] - b.point[0]);
	dist += (a.point[1] - b.point[1]) * (a.point[1] - b.point[1]);
	dist += (a.point[2] - b.point[2]) * (a.point[2] - b.point[2]);
	return sqrtf(dist);
}
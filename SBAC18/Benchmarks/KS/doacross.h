#include <stdlib.h>

int **sync_vec;
int n_nested, n_iter;

void alloca_vec() {
	sync_vec = (int **)malloc(n_iter*sizeof(int *));
	int i;

	for(i=0;i<n_iter;i++) {
		sync_vec[i] = (int *)malloc(n_nested*sizeof(int));
		memset(sync_vec[i], -1, n_nested*sizeof(int));
	}
}

void desalloca_vec() {
	int i;

	for(i=0;i<n_iter;i++) {
		free(sync_vec[i]);
	}

	free(sync_vec);
}

int vector_compare(int *A, int *B) {
	int i;

	if(n_nested == 1) {
		if(A[0] < B[0]) return -1;
	}
	else {
		for (i = 0; i < n_nested;i++) {
			if(A[i] < B[i]) return -1;
		}
	}

	return 0;
}

void clause_post(int *vec) {
	int i = vec[0], j;
	for (j = n_nested-1; j >= 0; j--)
		sync_vec[i][j] = vec[j];
}

void clause_wait(int *vec) {
//	if (outside_loop_bounds(vec)) return;
	int i = vec[0];
	while (vector_compare(sync_vec[i], vec) < 0) sleep(1/100000);
}

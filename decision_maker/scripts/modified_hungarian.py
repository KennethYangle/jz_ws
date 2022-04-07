import numpy as np
from copy import deepcopy


def unbalanced_assignment(matrix):
    org_mat = deepcopy(matrix)
    src_mat = deepcopy(matrix)
    row_n, col_n = np.shape(src_mat)
    if src_mat.min() < 0:
        src_mat -= src_mat.min()
    for c_i in range(col_n):
        col = src_mat[:, c_i]
        col -= min(col)
    for r_i in range(row_n):
        row = src_mat[r_i, :]
        row -= min(row)
    while True:
        row_lines, col_lines = coverage_lines(src_mat)
        least_line_num = len(row_lines) + len(col_lines)
        if least_line_num == row_n:
            break
        # get uncovered min
        _temp_mat = np.delete(src_mat, row_lines, axis=0)
        _uncovered_mat = np.delete(_temp_mat, col_lines, axis=1)
        uncovered_min = np.min(_uncovered_mat)
        # subtract this min from uncovered
        _row_unc_idx, _col_unc_idx = list(range(row_n)), list(range(col_n))
        for _r_i in row_lines:
            _row_unc_idx.remove(_r_i)
        for _c_i in col_lines:
            _col_unc_idx.remove(_c_i)
        for r_i in _row_unc_idx:
            for c_i in _col_unc_idx:
                src_mat[r_i, c_i] -= uncovered_min
        # add min to each intersection of lines
        for r_i in row_lines:
            for c_i in col_lines:
                src_mat[r_i, c_i] += uncovered_min

    def record_result(r, c):
        assignment_row.append(r)
        assignment_col.append(c)
    assignment_row = []
    assignment_col = []
    located_0_mat = np.ones(np.shape(src_mat))
    row_nonzero, col_nonzero = src_mat.nonzero()
    located_0_mat[row_nonzero, col_nonzero] = 0
    while np.sum(located_0_mat):
        normal_flag = False
        for r_i in range(row_n):
            row = located_0_mat[r_i, :]
            if sum(row) == 1:
                c_i = row.argmax()
                record_result(r_i, c_i)
                located_0_mat[:, c_i] = 0
                normal_flag = True
            else:
                continue
        if not normal_flag:
            for r_i in range(row_n):
                row = located_0_mat[r_i, :]  # type:np.ndarray
                if not sum(row):
                    continue
                elif sum(row) > 1:
                    nonzero_idx = row.nonzero()[0]
                    _min = org_mat[r_i, nonzero_idx[0]]
                    c_i = nonzero_idx[0]
                    for _c_i in nonzero_idx:
                        if org_mat[r_i, _c_i] < _min:
                            _min = org_mat[r_i, _c_i]
                            c_i = _c_i
                    record_result(r_i, c_i)
                    located_0_mat[:, c_i] = 0
    # # test output
    # result = np.zeros(src_mat.shape)
    # for r,c in zip(assignment_row, assignment_col):
    #     result[r, c] = 1
    # print(org_mat)
    # print(src_mat)
    # print(assignment_row, assignment_col)
    # print(result, np.sum(result*org_mat))
    return assignment_row, assignment_col


def coverage_lines(mat):
    src_mat = mat  # address
    row_n, col_n = np.shape(src_mat)
    # mark 0 with 1 or 2
    row_nz_i, col_nz_i = src_mat.nonzero()
    mark_1_mat = np.ones((row_n, col_n), int)
    mark_1_mat[row_nz_i, col_nz_i] = 0
    mark_2_mat = np.zeros((row_n, col_n), int)
    while True:
        mark_flag = False
        for r_i in range(row_n):
            row = mark_1_mat[r_i, :]  # type:np.ndarray
            if sum(row) == 1:  # single 0 in the row
                c_i = row.argmax()  # column of 0
                for r_i_c in range(row_n):  # mark
                    if r_i_c != r_i and mark_1_mat[r_i_c, c_i]:
                        # exist other 0 in the column
                        mark_1_mat[r_i_c, c_i] = 0
                        mark_2_mat[r_i_c, c_i] = 2
                        mark_flag = True  # marked flag
        for c_i in range(col_n):
            col = mark_1_mat[:, c_i]  # type:np.ndarray
            if sum(col) == 1:
                r_i = col.argmax()
                for c_i_r in range(col_n):
                    if c_i_r != c_i and mark_1_mat[r_i, c_i_r]:
                        mark_1_mat[r_i, c_i_r] = 0
                        mark_2_mat[r_i, c_i_r] = 2
                        mark_flag = True
        if not mark_flag:
            break
    # check row and column
    row_check_idx = []
    col_check_idx = []
    for r_i in range(row_n):
        if not sum(mark_1_mat[r_i, :]):
            row_check_idx.append(r_i)
    while True:
        check_flag = False
        for r_i in row_check_idx:
            for c_i in range(col_n):
                if mark_2_mat[r_i, c_i] and c_i not in col_check_idx:
                    col_check_idx.append(c_i)
                    check_flag = True
        for c_i in col_check_idx:
            for r_i in range(row_n):
                if mark_1_mat[r_i, c_i] and r_i not in row_check_idx:
                    row_check_idx.append(r_i)
                    check_flag = True
        if not check_flag:
            break
    # cover 0 with the least lines
    row_line_idx = []
    col_line_idx = deepcopy(col_check_idx)
    for r_i in range(row_n):
        if r_i not in row_check_idx:
            row_line_idx.append(r_i)
    # # test output
    # print(src_mat)
    # print(mark_1_mat)
    # print(mark_2_mat)
    # print(row_line_idx)
    # print(col_line_idx)
    # print(len(row_line_idx) + len(col_line_idx))
    # print('====================================')
    return row_line_idx, col_line_idx


if __name__ == '__main__':
    # matrix_test = np.array([[300, 250, 180, 320, 270, 190, 220, 260],
    #                         [290, 310, 190, 180, 210, 200, 300, 190],
    #                         [280, 290, 300, 190, 190, 220, 230, 260],
    #                         [290, 300, 190, 240, 250, 190, 180, 210],
    #                         [210, 200, 180, 170, 160, 140, 160, 180]])
    matrix_test = np.array([[1, 4, 3, 2, 3, 2, 1],
                            [3, 1, 2, 1, 4, 3, 2],
                            [1, 3, 3, 2, 3, 1, 4]])

    matrix_test = np.array([[4, 8, 2, 4, 6, 9, 4],
                            [1, 2, 5, 9, 7, 9, 1],
                            [1, 2, 5, 9, 7, 9, 1],
                            [1, 2, 5, 9, 7, 9, 1],
                            [1, 2, 5, 9, 7, 9, 1],
                            [1, 2, 5, 9, 7, 9, 1],
                            [3, 6, 4, 8, 2, 4, 3]])
    unbalanced_assignment(matrix_test)


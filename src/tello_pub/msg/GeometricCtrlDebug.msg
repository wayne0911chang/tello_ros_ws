# std_msgs/Header
# cost_func: h_0, h_1, h_2
# gradient vector in TSE(3): upper 12 entries of the 4x4 matrix
# [ 0, 1, 2, 3 ]
# [ 4, 5, 6, 7 ]
# [ 8, 9,10,11 ]
# [ 0, 0, 0, 0 ]

Header header
float64[3] cost_funcs
float64[12] grad_0
float64[12] grad_1
float64[12] grad_2
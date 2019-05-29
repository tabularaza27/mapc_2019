def mapSimiliarity(m1,m2,error_accepted):
	r_m1 = len(m1)
	c_m1 = len(m1[0])
	r_m2 = len(m2)
	c_m2 = len(m2[0])

	ret = []

	for i in range(0,len(m2)): # for every row
		for j in range(0,len(m2[i])): # for every column of the current row
				if m2[i][j] == m1[0][0]: # check the first element
					k = 0
					l = 0
					error_c = 0
					ok = True
					while k < r_m1 and ok == True: # scan the rows
						while l < c_m1 and ok == True: # scans the columns
							if (i+k) >= r_m2 or (j+l) >= c_m2 and error_c >= error_accepted: # if it is out of bound and no error accepted anymore, exit
								ok = False
							elif (i+k) >= r_m2 or (j+l) >= c_m2 and error_c < error_accepted: # if it is out of bound but error are accepted, go on and increase the error count
								error_c += 1
							else:
								if m2[i+k][j+l] != m1[k][l] and error_c >= error_accepted: # if it is not correct and no error accepted anymore, exit
									ok = False
								elif m2[i+k][j+l] != m1[k][l] and error_c < error_accepted: # if it is not correct but error are accepted, go on and increase the error count
									error_c += 1

							l += 1

						k += 1
						l = 0

					if (ok): # if found one, append to the list of returns
						ret.append([i,j])

	return ret

m1 = [ [0,0,0],
	   [0,1,0],
	   [0,0,7] ]

m2 = [ [0,0,0,0,0], 
	   [0,1,0,0,0],
	   [0,0,0,7,0],
	   [0,0,0,1,0],
	   [0,3,0,0,3] ]


s = mapSimiliarity(m1,m2,2)

if (s):
	print("yess from pos: " + str(s))
else:
	print(":(")	
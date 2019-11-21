e = '2..'
C = e.split('.')
len(C)
for i in range(len(C)):
	print(str(len(C))+" ")
	if(C[i]==""):
		C.pop(i)
print(C)

h = 90 + 20
h_list = []
for i in range(0, 40, 10):
    
    if i == 0:
        h_list.append(h)
        continue
    
    j = 1 
    h_list.append(h + j * i)
    j = -j
    h_list.append(h + j * i)

for h in h_list:
    print(h)
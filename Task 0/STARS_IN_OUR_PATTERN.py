T=int(input())

while T>0:
    N=int(input())
    for i in range(N,-1,-1):
        for j in range(1,i+1):
            if j%5==0 and j!=0:
                print('#',end='')
            else:
                print('*',end='')
        print("")
    T=T-1    
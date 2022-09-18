# cook your dish here
#   multiple of 3 add 3
#   multiple of 5 subtract 7 
#   sum between 3,4,5,6,7, 
  
  
testcases=int(input())

while testcases>0:
    l=int(input())
    a=[]
    a = list(map(int, input().split()))
    
    m=len(a)-1
    while(m>=0):
        print(a[m],end=" ")
        m=m-1
    print()
    
    j=0
    while(j+3<len(a)):
        print(a[j+3]+3,end=" ")
        j=j+3
        
    print()
    
    k=0
    while(k+5<len(a)):
        print(a[k+5]-7,end=" ")
        k=k+5
        
    print()
    
    print(a[3]+a[4]+a[5]+a[6]+a[7])
    
    testcases=testcases-1

    
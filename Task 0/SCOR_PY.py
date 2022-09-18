# cook your dish here
if __name__=='__main__':
    testcases=int(input())
    for i in range (0,testcases):
        students=int(input())
        max_marks=0
        max_name_list=[]
        flag=0
        for j in range (0,students):
            name,marks=input().split(" ")
            marks=float(marks)
            if(max_marks<marks):
                max_name=name 
            elif(max_marks==marks):
                max_name_list.append(max_name)
                max_name_list.append(name)
                flag=1
            max_marks=max(marks,max_marks)
            
        if(flag):
            
            max_name_list=sorted(max_name_list)
            for k in max_name_list:
                print(k)
            
        else:
            print(max_name)
            
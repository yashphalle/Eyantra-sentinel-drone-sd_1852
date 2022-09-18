# cook your dish here
testcases=int(input())

for i in range(0,testcases):
    items=int(input())
    dict={}
    sum=0
    for j in range(0,items):
        name,qty=input().split(" ")
        qty=int(qty)
        dict[name]=qty
        
    operation=int(input())
    for k in range(0,operation):
        op,name,qty=input().split(" ")
        
        if(op=="ADD"):
            if name in dict:
                dict[name]=dict[name]+int(qty)
                print("UPDATED Item "+name)
            else:
                dict[name]=int(qty)
                print("ADDED Item "+name)
        elif(op=="DELETE"):
            if name in dict:
                if(dict[name]>=int(qty)):
                    dict[name]=dict[name]-int(qty)
                    print("DELETED Item "+name)
                else:
                    print("Item "+name+" could not be DELETED")
            else:
                print("Item "+name+" does not exist")
                
    for i in dict:
        sum=sum+dict[i]
    
    print("Total Items in Inventory:",end=" ")  
    print(sum)
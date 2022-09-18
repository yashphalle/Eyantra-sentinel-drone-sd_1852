# cook your dish here
'''
This script is code stub for CodeChef problem code IFFOR1_PY
Filename:      IFFOR1_PY_solution.py
Created:       18/09/2023
Last Modified: 18/09/2023
Author:        e-Yantra Team
'''

# Main function
if __name__ == '__main__':
    
    # Take the T (test_cases) input
    test_cases = int(input())

    while test_cases>0:
        n=int(input())
        
        for i in range(0,n):
            if i==0:
                print(i+3,end=" ")
                
            elif i%2==0 and i!=0:
                print(2*i ,end=" ")
            
            elif i%2!=0 and i!=0:
                print(i*i ,end=" ")
        
        print("")            
        test_cases=test_cases-1        
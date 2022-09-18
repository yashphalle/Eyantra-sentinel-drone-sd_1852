
'''
This script is code stub for CodeChef problem code APLAM1_PY
Filename:      APLAM1_PY_solution.py
Created:       27/09/2021
Last Modified: 27/09/2021
Author:        e-Yantra Team
'''

# Import reduce module
from functools import reduce


# Function to generate the A.P. series
def generate_AP(a1, d, n):

    AP_series = []

    # Complete this function to return A.P. series
    for i in range (1,n+1):
        
        tn=a1+(i-1)*d
        AP_series.append(tn)
    
    return AP_series


# Main function
if __name__ == '__main__':
    
    # take the T (test_cases) input
    test_cases = int(input())

    # Write the code here to take the a1, d and n values
    
    for t in range (1,test_cases):
       
        a1,d,n=raw_input().split(" ")
       
        # Once you have all 3 values, call the generate_AP function to find A.P. series and print it
        AP_series = generate_AP(a1, d, n)

        # Using lambda and map fu[nctions, find squares of terms in AP series and print it
        sqr_AP_series = list(map(lambda n: n*n, AP_series))

        # Using lambda and reduce functions, find sum of squares of terms in AP series and print it
        sum_sqr_AP_series = reduce((lambda x, y: x + y),sqr_AP_series)
        
        for i in AP_series:
            print(i,end=" ")
        for i in sqr_AP_series:
            print(i,end=" ")
        print(sqr_AP_series)
        
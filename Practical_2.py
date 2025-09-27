# Create a model-based agent 
# Author: Aditya Kumar Roy

AIDS_Lab_05 = input("Is lab 5 is clean (yes/no):").lower()
AIDS_Lab_06 = input("Is lab 6 is clean (yes/no):").lower()

if AIDS_Lab_05 == "yes" and AIDS_Lab_06 =="no":
    print("clean AIds lab 6")
elif AIDS_Lab_05 =="no" and AIDS_Lab_06 =="yes":
    print("clean in Aids lab 5")
elif AIDS_Lab_05 =="no" and AIDS_Lab_06 =="no":
    print("Clean both the lab")
else:
    print("Both lab are clean and  ready to use")
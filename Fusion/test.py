import cv2

front = cv2.imread(r"./Pics/front.png")
fl = cv2.imread(r"./Pics/fl.png")
fr = cv2.imread(r"./Pics/fr.png")

cv2.imshow("front", front)
cv2.imshow("fl", fl)
cv2.imshow("fr", fr)

cv2.waitKey(0)
cv2.destroyAllWindows()





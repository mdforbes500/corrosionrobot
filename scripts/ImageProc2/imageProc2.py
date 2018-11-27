from PIL import Image
import sys
from PIL import ImageFilter



image = sys.argv[1]
pic = Image.open(image, 'r')
pixel_number_wide = pic.size[0]
image_width = float(sys.argv[2])

pixel_size = (image_width/float(pixel_number_wide))**2

print "Pixel size: {0} " .format(pixel_size)
pixels = list(pic.getdata())
counter = 0
corCount = 0
for x in pixels:
	if(x[0]+x[1]+x[2] >= 0):
		counter+=1
	if(x[0]+x[1]+x[2] < 300):
		corCount+=1
		
print "Number of pixels: {0}\n" .format(counter)
print "Number of corrosion pixels: {0}\n" .format(corCount)
corrosion_area = pixel_size*corCount
print "Total Area of Corrosion: {0} Square Centimeters" .format(corrosion_area)

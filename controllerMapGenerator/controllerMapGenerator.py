from pathlib import Path
import re
from PIL import Image, ImageFont
from PIL import ImageDraw

def getRobotContainer():
    filePath = Path("src/main/java/frc/robot/RobotContainer.java")

    with open(filePath, "r", encoding="utf-8") as file:
        robotContainer = file.read()
    return robotContainer

def getButtonMapping(controller, button):
    pattern = rf"{controller}\.{button}\(\)(?:\s*//\s*(.*))?"
    matches = list(re.finditer(pattern, getRobotContainer()))

    if not matches:
        return None

    comment = ""

    for match in matches:
        comment += match.group().split("//")[1] + "\n"

    return comment.strip()


img = Image.open('controllerMapGenerator/XboxNavigation.png')
font = ImageFont.truetype("controllerMapGenerator/Electrolize-Regular.ttf", size=36)
titleFont = ImageFont.truetype("controllerMapGenerator/Electrolize-Regular.ttf", size=48)
# Call draw Method to add 2D graphics in an image
I1 = ImageDraw.Draw(img)

# Add Text to an image
I1.text((630, 120), getButtonMapping('driverController','leftTrigger'), fill=(0, 0, 0), font=font, anchor="mm")
I1.text((1200, 120), getButtonMapping('driverController','rightTrigger'), fill=(0, 0, 0), font=font, anchor="mm")
I1.text((1450, 300), getButtonMapping('driverController','y'), fill=(0, 0, 0), font=font)
I1.text((1450, 480), getButtonMapping('driverController','b'), fill=(0, 0, 0), font=font)
I1.text((1450, 560), getButtonMapping('driverController','x'), fill=(0, 0, 0), font=font)
#I1.text((1450, 820), getButtonMapping('driverController','a'), fill=(0, 0, 0), font=font)
I1.text((100,510), "Drive - Translate", fill=(0, 0, 0), font=font, align="rm")
I1.text((1020,920), "Drive - Rotate", fill=(0, 0, 0), font=font, align="mm")
I1.text((750, 10), "Driver Controller Map", fill=(0, 0, 0), font=font, align="mm")


# Display edited image
img.show()

# Save the edited image
img.save("controllerMapGenerator/driverMap.png")
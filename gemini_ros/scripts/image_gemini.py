import google.generativeai as genai
from PIL import Image

# Set your API key
genai.configure(api_key="AIzaSyCfzzJMRAR6f-aNdq7JhCoWcs6pmz_mfW8")

# Load an image
image_path = "img1.jpg"   # Change to your image file
img = Image.open(image_path)

# Choose the Gemini model
model = genai.GenerativeModel("gemini-1.5-flash")

# Generate content with text + image
response = model.generate_content(
    ["Describe the chair in detail: color, type, and position of the person sitting on it.", img]
)

# Print output
print(response.text)

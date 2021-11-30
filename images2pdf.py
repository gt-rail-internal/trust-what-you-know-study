from os import path
from PIL import Image

def load_and_convert_PIL(image_path):
    png = Image.open(image_path)
    png.thumbnail((595, 842), Image.ANTIALIAS) #a4
    png.load() # required for png.split()

    if len(png.split()) > 2:
        background = Image.new("RGB", png.size, (255, 255, 255))
        background.paste(png, mask=png.split()[3]) # 3 is the alpha channel

        return background

    else:
        return png

def save_pdf(list, round=0):

    path_list = ['./server/static/img/hf_{}_s.png'.format(l) for l in list]
    image1 = load_and_convert_PIL(path_list[0])
    image_list = [load_and_convert_PIL(ig) for ig in path_list[1:]]

    image1.save('./solutions_pdf/solutions_round_{}.pdf'.format(round), save_all=True, append_images=image_list)

if __name__ == '__main__':

    save_pdf([40, 41, 20, 21, 30, 31, 50, 51, 10, 11], round=0)
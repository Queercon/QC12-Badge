import os, sys, argparse, re, itertools, base64

from collections import OrderedDict

from glob import glob
from ConfigParser import ConfigParser

from PIL import Image, ImageDraw, ImageFont
import PIL.ImageOps
import PIL.ImageChops

uniform_sprite_size = 0

SPRITE_WIDTH = 64
SPRITE_HEIGHT = 64
 
HEAD_HEIGHT = 22
BODY_HEIGHT = 24
FEET_HEIGHT = 18

DEFAULT_SPEED = 3

head_clip = 64
legs_clip = 0

is_a_number = re.compile("[0-9]+")


def get_bits(paths, part=None):
    segs = []
    for path in paths:
        img, mask = adjust_image(Image.open(path))
        seg = Image.new('RGBA', (SPRITE_WIDTH, SPRITE_HEIGHT), (0,0,0,0))
        seg.paste(img, (0, 0), mask)
        
        out_str = "{0b"
        index = 0
        sprite_size = 0
        for pixel in list(seg.getdata()):
            if 'head' in path and sprite_size/64 == head_clip:
                break
            if 'legs' in path and sprite_size/64 < legs_clip:
                sprite_size += 1
                continue
            if index == SPRITE_WIDTH:
                out_str += ", \n     0b"
                index = 0
            if index and index % 8 == 0:
                out_str += ", 0b"
            out_str += "1" if sum(pixel[:-1]) else "0" # Everything but alpha...
            index += 1
            sprite_size += 1
        out_str += "},"
        segs.append(out_str)
    if part and part in ["head", "legs"]:
        segs.append("{0, }, // blank!")
    elif part and part == "body":
        for path in sorted(glob(('infant/torso/*.png')), key=lambda a: int(is_a_number.findall(a)[0])): # kludge... TODO ?
            img, mask = adjust_image(Image.open(path))
            seg = Image.new('RGBA', (SPRITE_WIDTH, SPRITE_HEIGHT), (0,0,0,0))
            seg.paste(img, (0, 0), mask)
            
            out_str = "{0b"
            index = 0
            sprite_size = 0
            for pixel in list(seg.getdata()):
                if 'head' in path and sprite_size/64 == head_clip:
                    break
                if 'legs' in path and sprite_size/64 < legs_clip:
                    sprite_size += 1
                    continue
                if index == SPRITE_WIDTH:
                    out_str += ", \n     0b"
                    index = 0
                if index and index % 8 == 0:
                    out_str += ", 0b"
                out_str += "1" if sum(pixel[:-1]) else "0" # Everything but alpha...
                index += 1
                sprite_size += 1
            out_str += "},"
            segs.append(out_str)
    
    return segs
    
def get_clipping_areas(dirs):
    lowest_head = 0
    highest_body_top = 64
    lowest_body_bottom = 0
    tallest_legs = 64
    for d in dirs:
        head_files = sorted(glob(os.path.join(d, 'head', '*.png')), key=lambda a: int(is_a_number.findall(a)[0]))
        body_files = sorted(glob(os.path.join(d, 'torso', '*.png')), key=lambda a: int(is_a_number.findall(a)[0]))
        legs_files = sorted(glob(os.path.join(d, 'legs', '*.png')), key=lambda a: int(is_a_number.findall(a)[0]))
        for headfile in head_files:
            head, head_mask = adjust_image(Image.open(headfile))
            if head_mask.getbbox()[3] > lowest_head:
                lowest_head = head_mask.getbbox()[3]
        for legsfile in legs_files:
            legs, legs_mask = adjust_image(Image.open(legsfile))
            if legs_mask.getbbox()[1] < tallest_legs:
                tallest_legs = legs_mask.getbbox()[1]
        for bodyfile in body_files:
            body, body_mask = adjust_image(Image.open(bodyfile))
            if body_mask.getbbox()[1] < highest_body_top:
                highest_body_top = body_mask.getbbox()[1]
            if body_mask.getbbox()[3] > lowest_body_bottom:
                lowest_body_bottom = body_mask.getbbox()[3]
    return lowest_head, tallest_legs

def make_outputs(head_paths, body_paths, legs_paths):
    parts = dict(
        heads = get_bits(head_paths, "head"),
        bodies = get_bits(body_paths, "body"),
        legs = get_bits(legs_paths, "legs")
    )
    
    for partname, bits in parts.items():
        size = {'heads': head_clip*8, 'bodies': 512, 'legs': (64-legs_clip)*8}[partname]
        print 'const uint8_t %s_pixels[%d][%d] = {' % (partname, len(bits), size)
        print '   ',
        print '\n    '.join(bits)
        print '};'
    
    print
    
    for partname, bits in parts.items():
        print "const tImage %s[%d] = {" % (partname, len(bits))
        for i in range(len(bits)):
            print "   {IMAGE_FMT_1BPP_UNCOMP, 64, %d, 2, palette_bw, %s_pixels[%d]}," % (
                {'heads': head_clip, 'bodies': 64, 'legs': 64-legs_clip}[partname],
                partname,
                i
            )
        print '};'
        print
    
    return parts['heads'], parts['bodies'], parts['legs']

def get_tops(head_path, body_path, legs_path, heights):
    if head_path == body_path == legs_path == None:
        return 0,0,0
    
    head, head_mask = adjust_image(Image.open(head_path))
    body, body_mask = adjust_image(Image.open(body_path))
    legs, legs_mask = adjust_image(Image.open(legs_path))
    
    # This is the amount of blank space at the bottom of the legs:
    squat_amount = legs.size[1] - legs_mask.getbbox()[3]
    
    # Now we place the legs; the bottom of the bounding box needs to be aligned
    # with the bottom of our sprite canvas:
    legs_y = SPRITE_HEIGHT - legs_mask.getbbox()[3]
    
    # Move the torso DOWN by the height of its corresponding legs.
    #  Then the torso will be against the bottom of the screen.
    # Then move it UP by the height of the legs we're using.
    # And DOWN by squat_amount.
    body_y = heights[3] + squat_amount - heights[2]
    
    # The head is designed to fit on something with a body+legs height
    #  of heights[4] + heights[5].
    # It's actually on something with body+legs height of heights[1]+heights[2].
    head_y = squat_amount + (heights[4]+heights[5]-heights[2]-heights[1])
    
    return head_y, body_y, legs_y

animations = []

done_combos = []

combo_image = Image.new('RGBA', (SPRITE_WIDTH*7, SPRITE_HEIGHT*10), (0,0,0,255))

def show_char(head_files, body_files, legs_files, heights, indices, thumb_id):
    if indices not in done_combos and head_files[indices[0]] and body_files[indices[1]] and legs_files[indices[2]]:
        head_img, head_mask = adjust_image(Image.open(head_files[indices[0]]))
        body_img, body_mask = adjust_image(Image.open(body_files[indices[1]]))
        legs_img, legs_mask = adjust_image(Image.open(legs_files[indices[2]]))
        
        hy,by,ly = get_tops(head_files[indices[0]], body_files[indices[1]], legs_files[indices[2]], heights)
        
        sprite = Image.new('RGBA', (SPRITE_WIDTH, SPRITE_HEIGHT), (0,0,0,0))
        sprite.paste(head_img, (0,hy), head_mask)
        sprite.paste(body_img, (0,by), body_mask)
        sprite.paste(legs_img, (0,ly), legs_mask)
        
        combo_image.paste(head_img, (SPRITE_WIDTH*(len(done_combos)%7), SPRITE_HEIGHT*(len(done_combos)/7) + hy), head_mask)
        combo_image.paste(body_img, (SPRITE_WIDTH*(len(done_combos)%7), SPRITE_HEIGHT*(len(done_combos)/7) + by), body_mask)
        combo_image.paste(legs_img, (SPRITE_WIDTH*(len(done_combos)%7), SPRITE_HEIGHT*(len(done_combos)/7) + ly), legs_mask)
        
        sprite_image, sprite_mask = adjust_image(sprite)
        done_combos.append(indices)
        
        #if len(done_combos) == 66:
        if thumb_id or thumb_id==0:#
            combo_image.save(os.path.join("thumbs", "spritesheets", "all_%03d.png") % thumb_id)
        else:
            combo_image.show()

def make_thumbs(thumb_id, head_files, body_files, legs_files, heights):
    head_img, head_mask = adjust_image(Image.open(head_files[0]))
    body_img, body_mask = adjust_image(Image.open(body_files[0]))
    legs_img, legs_mask = adjust_image(Image.open(legs_files[0]))
    
    hy,by,ly = get_tops(head_files[0], body_files[0], legs_files[0], heights)
    
    sprite = Image.new('RGBA', (SPRITE_WIDTH, SPRITE_HEIGHT), (0,0,0,0))
    sprite.paste(head_img, (0,hy), head_mask)
    sprite.paste(body_img, (0,by), body_mask)
    sprite.paste(legs_img, (0,ly), legs_mask)
    
    text_image = Image.new('RGBA', (SPRITE_HEIGHT+20, SPRITE_WIDTH), (255,255,255,0))
    f = ImageFont.truetype(font="Consolas.ttf", size=10)
    td = ImageDraw.Draw(text_image)
    
    sprite_image, sprite_mask = adjust_image(sprite)
    sprite_image.save(os.path.join("thumbs", "%03d_f.png") % thumb_id)
    thumbnail = Image.new('RGBA', (SPRITE_WIDTH, SPRITE_HEIGHT+20), (0,0,0,0))
    f = ImageFont.truetype(font="Consolas.ttf", size=20)
    td = ImageDraw.Draw(thumbnail)
    width = td.textsize(str(thumb_id), font=f)[0]
    thumbnail.paste(text_image.rotate(-90), (0,0))
    thumbnail.paste(adjust_image(sprite)[0], (0,0), sprite_mask)
    td.text(((64-width)/2,68), str(thumb_id), font=f, fill=(0,0,0))
    thumbnail.save(os.path.join("thumbs", "label_%03d.png") % thumb_id)
    
    b64label = ""
    with open(os.path.join("thumbs", "label_%03d.png") % thumb_id, "rb") as im:
        b64label = base64.b64encode(im.read())
    
    label_str = """<?xml version="1.0" encoding="utf-8"?>
        <DieCutLabel Version="8.0" Units="twips">
        <PaperOrientation>Portrait</PaperOrientation>
        <Id>Small30332</Id>
        <PaperName>30332 1 in x 1 in</PaperName>
        <DrawCommands>
        <RoundRectangle X="0" Y="0" Width="1440" Height="1440" Rx="180" Ry="180" />
        </DrawCommands>
        <ObjectInfo>
        <ImageObject>
        <Name>GRAPHIC</Name>
        <ForeColor Alpha="255" Red="0" Green="0" Blue="0" />
        <BackColor Alpha="0" Red="255" Green="255" Blue="255" />
        <LinkedObjectName></LinkedObjectName>
        <Rotation>Rotation0</Rotation>
        <IsMirrored>False</IsMirrored>
        <IsVariable>False</IsVariable>
        <Image>"""
    label_str += b64label
    label_str += """</Image>
        <ScaleMode>Uniform</ScaleMode>
        <BorderWidth>0</BorderWidth>
        <BorderColor Alpha="255" Red="0" Green="0" Blue="0" />
        <HorizontalAlignment>Center</HorizontalAlignment>
        <VerticalAlignment>Center</VerticalAlignment>
        </ImageObject>
        <Bounds X="82" Y="144" Width="1301" Height="1210" />
        </ObjectInfo>
        </DieCutLabel>
        """
    
    with open(os.path.join("thumbs", "labels", "label%03d.label" % thumb_id), 'w') as lf:
        lf.write(label_str)
    

def main(inifile, head_dir, body_dir, legs_dir, show, thumb_id=False):
    parser = ConfigParser()
    parser.read(inifile)
    head_files = sorted(glob(os.path.join(head_dir, 'head', '*.png')), key=lambda a: int(is_a_number.findall(a)[0]))
    body_files = sorted(glob(os.path.join(body_dir, 'torso', '*.png')), key=lambda a: int(is_a_number.findall(a)[0]))
    legs_files = sorted(glob(os.path.join(legs_dir, 'legs', '*.png')), key=lambda a: int(is_a_number.findall(a)[0]))
        
    heights = [HEAD_HEIGHT, BODY_HEIGHT, FEET_HEIGHT, FEET_HEIGHT, BODY_HEIGHT, FEET_HEIGHT]
    
    if os.path.isfile(os.path.join(body_dir, 'torso', 'height')):
        with open(os.path.join(body_dir, 'torso', 'height')) as height_file:
            heights[1] = int(height_file.readline())
    
    if os.path.isfile(os.path.join(legs_dir, 'legs', 'height')):
        with open(os.path.join(legs_dir, 'legs', 'height')) as height_file:
            heights[2] = int(height_file.readline())
    
    if os.path.isfile(os.path.join(body_dir, 'legs', 'height')):
        with open(os.path.join(body_dir, 'legs', 'height')) as height_file:
            heights[3] = int(height_file.readline())
    
    if os.path.isfile(os.path.join(head_dir, 'torso', 'height')):
        with open(os.path.join(head_dir, 'torso', 'height')) as height_file:
            heights[4] = int(height_file.readline())
    
    if os.path.isfile(os.path.join(head_dir, 'legs', 'height')):
        with open(os.path.join(head_dir, 'legs', 'height')) as height_file:
            heights[5] = int(height_file.readline())
    
    make_outputs(head_files, body_files, legs_files) # Prints directly.
    
    if thumb_id or thumb_id==0 or show:
        make_thumbs(thumb_id, head_files, body_files, legs_files, heights)
    
    index_offset = 0
    index = 0
    longest_anim_buffer = 0
    
    move = 0
    
    head_files.append(None)
    body_files.append(None)
    body_files.append(None)
    body_files.append(None)
    legs_files.append(None)
    
    for anim_name in parser.sections():
        # Note: The in animation must be first, followed by loop, then out.
        if ":loop" in anim_name:
            anim['loop_start_index'] = index_offset = index
        elif ":out" in anim_name:
            anim['loop_end_index'] = index_offset = index
        else:
            anim = dict(looped=(":in" in anim_name), name=anim_name.split(':')[0], parts=OrderedDict(heads=[], bodies=[], legs=[]), offsets=OrderedDict(head=[], body=[], legs=[]), moves=[])
            anim['speed'] = DEFAULT_SPEED
            animations.append(anim)
            index_offset = 0
        for frame in parser.items(anim_name):
            if is_a_number.match(frame[0]):
                # It's a frame number.
                
                # TODO: assert that the frame numbers are correct please.
                # TODO: assert indices are in bounds
                # TODO: assert the the filename endswith index-1
                
                head_index, body_index, legs_index = map(int, frame[1].split(','))
                anim['parts']['heads'].append(head_index-1)
                anim['parts']['bodies'].append(body_index-1)
                anim['parts']['legs'].append(legs_index-1)
                
                if show:
                    show_char(head_files, body_files, legs_files, heights, (head_index-1, body_index-1, legs_index-1), thumb_id)
                
                h,b,t = get_tops(head_files[head_index-1], body_files[body_index-1], legs_files[legs_index-1], heights)
                
                anim['offsets']['head'].append(h)
                anim['offsets']['body'].append(b)
                anim['offsets']['legs'].append(t)
                
                index = int(frame[0]) + index_offset
                anim['moves'].append(move)
                move = 0
            elif frame[0].startswith("up"): # 00xxxxxx - up
                assert int(frame[1]) < 65
                move = frame[1]
            elif frame[0].startswith("down"): # 01xxxxxx - down
                assert int(frame[1]) < 65
                move = 0x40 + int(frame[1])
            elif frame[0].startswith("left"): # 10xxxxxx - left
                move = 0x80 + int(frame[1])
                assert int(frame[1]) < 65
            elif frame[0].startswith("right"): # 11xxxxxx - right
                move = 0xC0 + int(frame[1])
                assert int(frame[1]) < 65
            elif frame[0] == "speed":
                anim['speed'] = int(frame[1])
        if index > longest_anim_buffer:
            longest_anim_buffer = index
    
    for anim in animations:
        assert len(anim['parts']['heads']) == len(anim['parts']['bodies']) == len(anim['parts']['legs']) == len(anim['moves'])
        anim['len'] = len(anim['parts']['heads'])
        
    """
    typedef struct {
        uint8_t looped;
        uint8_t loop_start;
        uint8_t loop_end;
        uint8_t len;
        uint8_t speed;
        const tImage** head_images;
        const tImage** body_images;
        const tImage** legs_images;
        const uint8_t* head_tops;
        const uint8_t* body_tops;
        const uint8_t* legs_tops;
        const uint8_t* movement;
    } qc12_anim_t;
    """
    
    print "uint8_t no_moves[%d] = {%s};" % (longest_anim_buffer, '0')
    print
    
    for anim in animations:
        no_moves = all(i == 0 for i in anim['moves'])
        anim['moves'] = map(str, anim['moves'])
        
        print "// " + anim['name']
        
        for partname, indices in anim['parts'].items():
            print "const uint8_t %s_%s[%d] = {" % (anim['name'], partname, anim['len']),
            print "" + ', '.join(map(str, indices)),
            print "};"
        
        for partname, tops in anim['offsets'].items():
            print "const int8_t %s_%s_tops[%d] = {" % (anim['name'], partname, anim['len']),
            print "" + ', '.join(map(str, tops)),
            print "};"
                
        if not no_moves:
            print "const uint8_t %s_moves[%d] = { %s };" % (
                anim['name'],
                anim['len'],
                ", ".join(anim['moves'])
            )
            print ""
        
        print "const qc12_anim_t %s = {" % anim['name']
        print "\t%d, // Looped?" % (int(anim['looped']) if 'looped' in anim else 0)
        print "\t%d, // Loop start index" % (anim['loop_start_index'] if 'loop_start_index' in anim else 0)
        print "\t%d, // Loop end index" % (anim['loop_end_index'] if 'loop_end_index' in anim else anim['len'])
        print "\t%d, // Length" % anim['len']
        print "\t%d, // Speed" % anim['speed']
        for partname in ['heads', 'bodies', 'legs']:
            print "\t%s_%s, //%s" % (anim['name'], partname, partname)
        for partname in ['head', 'body', 'legs']:
            print "\t%s_%s_tops, //%s" % (anim['name'], partname, partname)
        if no_moves:
            print "\tno_moves, // Moves"
        else:
            print "\t%s_moves, // Moves" % anim['name']
        print "};"
        print
        
    print "// anim_buffer_alloc = %d" % longest_anim_buffer
    print
    print "// For the animation demo:"
    print
    print "const qc12_anim_t *demo_anims[] = {"
    print "   ",
    print ", ".join(("&"+anim['name']) for anim in animations)
    print "};"
    print "const uint8_t demo_anim_count = " + str(len(animations)) + ";"

def adjust_image(image):
    assert image.mode == 'RGBA'
    a = image.split()[-1] # Alpha channel
    r = image.split()[0] # Red channel
    r_i = PIL.ImageOps.invert(r)
    return Image.merge('RGBA', (r_i,r_i,r_i,a)), a

if (__name__ == '__main__'):
    parser = argparse.ArgumentParser(description='Generate a QC12 badge character sprite set.')
    parser.add_argument('-s', '--show', action='store_true', help="Display the"
                        " images as they are generated")
    parser.add_argument('-f', '--fixini', action='store_true', help="Fix the"
                        " format of the ini file.")
    parser.add_argument('config', help='Path to config file specifying the'
                                       'animations to make')
    parser.add_argument('plays', help='Path to config file specifying the'
                                       'play animations to make')
    group = parser.add_mutually_exclusive_group()
    group.add_argument('-i', '--id', help="Generate from badge ID", type=int)
    parts = group.add_argument_group("Body parts")
    parts.add_argument('-H', '--head', metavar='head_dir', type=str, 
                        help="Character directory to look in for the sprite's "
                             "head (must have a `head' subdirectory)")
    parts.add_argument('-B', '--body', metavar='body_dir', type=str, 
                        help="Character directory to look in for the sprite's "
                             "body (must have a `body' subdirectory)")
    parts.add_argument('-L', '--legs', metavar='legs_dir', type=str, 
                        help="Character directory to look in for the sprite's "
                             "legs (must have a `legs' subdirectory)")
    args = parser.parse_args()
    
    use_file = args.config
    
    if args.fixini:
        filestring = ""
        with open(args.config) as configfile:
            for line in configfile:
                filestring += line
        filestring += "\n"
        with open(args.plays) as configfile:
            for line in configfile:
                filestring += line
        left_num = 0
        right_num = 0
        up_num = 0
        down_num = 0
        # TODO: should not match animation names.
        # TODO: this should search for the highest one.
        while filestring.find("left: ") != -1 or \
              filestring.find("right: ") != -1 or \
              filestring.find("up: ") != -1 or \
              filestring.find("down: ") != -1:
            filestring = filestring.replace("left: ", "left%d: " % left_num, 1)
            filestring = filestring.replace("right: ", "right%d: " % right_num, 1)
            filestring = filestring.replace("up: ", "up%d: " % up_num, 1)
            filestring = filestring.replace("down: ", "down%d: " % down_num, 1)
            left_num = right_num = up_num = down_num = down_num+1
            
        with open(args.config + '.tmp', 'w') as configfile:
            configfile.write(filestring)
        use_file = args.config + '.tmp'
    
    uber_dirs = ["uber_astronaut", "uber_bender", "uber_black_mage", "uber_blackhat", "uber_demon", "uber_human", "uber_minecraft", "uber_minion", "uber_shark", "uber_stig", "uber_ness", "uber_corrinsan"]
    human_dirs = ["alien", "bear", "human", "lizard", "octopus", "robot"]
    human_computed_ids = [19,27,28,30,31,32,33,52,53,54,56,60,61,62,72,75,76,77,78,80,81,82,85,86,87,90,92,99,103,104,106,111,112,114,116,117,120,122,124,127,130,133,136,137,138,141,143,145,146,149,150,151,152,154,157,158,163,164,169,180,185,187,188,194,195,196,197,198,199,200,205,208,209,211,217,220,228,229,230,231]
    
    if 'id' in args:
        if (args.id >= 12):
            l = list(itertools.product(human_dirs, repeat=3))
            head, body, legs = l[(human_computed_ids[(args.id-15) % len(human_computed_ids)]-15) % len(l)]
        else:
            head = body = legs = uber_dirs[args.id]
       
    all_dirs = uber_dirs + human_dirs

    head_clip, legs_clip = get_clipping_areas(all_dirs)
    
    print "const uint8_t legs_clip_offset = %d;" % legs_clip
    print
    
    main(use_file, args.head or head, args.body or body, args.legs or legs, args.show, thumb_id=args.id)
    
    print
    print "// Head size: %d, leg size: %d" % (head_clip, legs_clip)
    if args.id < 15:
        print "// This is %s" % uber_dirs[args.id]
        
        
    if args.id < 15:
        sprite_id = args.id
    else:
        sprite_id = args.id - 15
        sprite_id = sprite_id % len(list(itertools.product(human_dirs, repeat=3)))
        sprite_id = sprite_id + 15
        
    print "const uint8_t my_sprite_id = %d;" % sprite_id
    
    if args.fixini:
        os.remove(use_file)
        pass
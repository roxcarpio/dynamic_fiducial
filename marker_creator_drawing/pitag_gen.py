#!/usr/bin/env python#!/usr/bin/env python

import svgwrite
import os
from svgwrite import cm, mm
import sys
import rsvg
import textwrap

##################### TO BE MODIFIED #################
AB0 = 0.40
AC0 = 0.60
AB1 = 0.30
AC1 = 0.70

circle_radius = 1

######################################################

circle_clearance = 0.2
output_file = './static/selected'

def pitag_drawer(arg):

    size_outer_square = arg

    marker_size = size_outer_square - circle_radius*2 -circle_clearance*2

    file_name = os.path.splitext(output_file)[0]

    w, h = '100%', '100%'
    dwg = svgwrite.Drawing(filename=file_name+'.svg', size=(w, h), debug=True)
    dwg.add(dwg.rect(insert=(0,0), size=(w, h), fill='white', stroke='none'))


    #CrossRatios 0 - 1.0 relative to marker_size

    d_line0_AB = marker_size * AB0 #AB
    d_line0_BD = marker_size - marker_size *AB0 #BD
    d_line0_AC = marker_size * AC0 #//AC
    d_line0_CD = marker_size - marker_size * AC0 #//CD
    cross_ration_0 = (d_line0_AB/d_line0_BD)/(d_line0_AC/d_line0_CD)

    d_line1_AB = marker_size * AB1 #AB
    d_line1_BD = marker_size - marker_size *AB1 #BD
    d_line1_AC = marker_size * AC1 #;//AC
    d_line1_CD = marker_size - marker_size * AC1 #;//CD
    cross_ration_1 = (d_line1_AB/d_line1_BD)/(d_line1_AC/d_line1_CD)

    print "cross_ration 0:  {0}".format(cross_ration_0)
    print "cross_ration 1:  {0}".format(cross_ration_1)
    print "delta:{0}".format(cross_ration_0 / cross_ration_1)

    if cross_ration_0 < cross_ration_1:
        print "Error, Crossratio 0 must be greater that Crossratio 1"
        sys.exit()

    CR_Line0_AB = AB0
    CR_Line0_AC = AC0
    CR_Line1_AB = AB1
    CR_Line1_AC = AC1

    #center of the marker
    cx = cy = circle_clearance + circle_radius + marker_size/2

    #Squares for marker size reference
    size_inner_square = marker_size-circle_radius*2-circle_clearance*2
    corner_inner_square = 2*circle_radius + 2*circle_clearance
    size_outer_square = marker_size+circle_radius*2+circle_clearance*2

    #We create a group for the whole marker
    marker = dwg.defs.add(dwg.g(id='marker'))

    #Outer square
    marker.add(svgwrite.shapes.Rect((0*cm,0*cm),(size_outer_square*cm,size_outer_square*cm), fill='white', stroke='black'))

    #inner square
    marker.add(svgwrite.shapes.Rect((corner_inner_square*cm,corner_inner_square*cm),(size_inner_square*cm,size_inner_square*cm), fill='none', stroke='black'))

    x1 = y1 = circle_radius + circle_clearance
    x2 = y2 = circle_radius + circle_clearance + marker_size

    #lets draw the marker corner circles
    top_left_corner = (x1*cm,y1*cm)
    bottom_left_corner = (x1*cm,y2*cm)
    top_right_corner = (x2*cm,y1*cm)
    bottom_right_corner = (x2*cm,y2*cm)
    marker.add(svgwrite.shapes.Circle(top_left_corner, circle_radius*cm))
    marker.add(svgwrite.shapes.Circle(bottom_left_corner, circle_radius*cm))
    marker.add(svgwrite.shapes.Circle(top_right_corner, circle_radius*cm))
    marker.add(svgwrite.shapes.Circle(bottom_right_corner, circle_radius*cm))

    #Now we draw the Cross related circles for line 0
    marker.add(svgwrite.shapes.Circle(((x1+marker_size*CR_Line0_AB)*cm,y1*cm), circle_radius*cm))
    marker.add(svgwrite.shapes.Circle((x1*cm,(y1+marker_size*CR_Line0_AB)*cm), circle_radius*cm))

    marker.add(svgwrite.shapes.Circle(((x1+marker_size*CR_Line0_AC)*cm,y1*cm), circle_radius*cm))
    marker.add(svgwrite.shapes.Circle((x1*cm,(y1+marker_size*CR_Line0_AC)*cm), circle_radius*cm))

    #Now we draw the Cross related circles for line 1
    marker.add(svgwrite.shapes.Circle(((x1+marker_size*CR_Line1_AB)*cm,y2*cm), circle_radius*cm))
    marker.add(svgwrite.shapes.Circle((x2*cm,(y1+marker_size*CR_Line1_AB)*cm), circle_radius*cm))

    marker.add(svgwrite.shapes.Circle(((x1+marker_size*CR_Line1_AC)*cm,y2*cm), circle_radius*cm))
    marker.add(svgwrite.shapes.Circle((x2*cm,(y1+marker_size*CR_Line1_AC)*cm), circle_radius*cm))

    d_line0_AB = marker_size * AB0 #AB
    d_line0_BD = marker_size - marker_size *AB0 #BD
    d_line0_AC = marker_size * AC0 #//AC
    d_line0_CD = marker_size - marker_size * AC0 #//CD
    cross_ration_0 = (d_line0_AB/d_line0_BD)/(d_line0_AC/d_line0_CD)

    d_line1_AB = marker_size * AB1 #AB
    d_line1_BD = marker_size - marker_size *AB1 #BD
    d_line1_AC = marker_size * AC1 #;//AC
    d_line1_CD = marker_size - marker_size * AC1 #;//CD

    #red do in the Center of the marker
    marker.add(svgwrite.shapes.Circle((cx*cm,cy*cm), 0.05*cm, fill='red'))

    marker.add(dwg.line((cx*cm, cy*cm), ((cx+2)*cm, cy*cm), stroke=svgwrite.rgb(10, 10, 16, '%')))
    marker.add(dwg.line((cx*cm, cy*cm), (cx*cm, (cy-2)*cm), stroke=svgwrite.rgb(10, 10, 16, '%')))
    marker.add(dwg.text('x', insert=((cx+2.1)*cm, (cy)*cm), fill='black'))
    marker.add(dwg.text('y', insert=((cx)*cm, (cy-2.2)*cm), fill='black'))

    u = dwg.use(marker, insert=(0*cm,0*cm))

    dwg.add(u)

    dwg.save()

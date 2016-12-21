#!/usr/bin/env python#!/usr/bin/env python

import svgwrite
import os
from svgwrite import cm, mm
import sys

import rsvg
import textwrap
import math


def aruco_drawer_id0(arg):

    output_file = './static/selected'
    file_name = os.path.splitext(output_file)[0]

    marker_size = arg

    w, h = marker_size, marker_size
    dwg = svgwrite.Drawing(filename=file_name + '.svg', size=(w*cm, h*cm), debug=True)
    dwg.add(dwg.rect(insert=(0,0), size=(w*cm, h*cm), fill='black', stroke='none'))

    #center of the marker
    cx = cy = marker_size/2

    #Squares for marker size reference
    corner_square = 0.137 * marker_size + 0.0196
    size_inner_square = 0.725 * marker_size - 0.392
    size_id_square_x = corner_square
    size_id_square_y = size_inner_square

    #We create a group for the whole marker
    marker = dwg.defs.add(dwg.g(id='marker'))

    #ID000
    #inner square
    marker.add(svgwrite.shapes.Rect((corner_square*cm,corner_square*cm),(size_inner_square*cm,size_inner_square*cm), fill='black'))

    #id square
    marker.add(svgwrite.shapes.Rect((corner_square*cm,corner_square*cm),(size_id_square_x*cm,size_id_square_y*cm), fill='white'))

    u = dwg.use(marker, insert=(0*cm,0*cm))
    dwg.add(u)
    dwg.save()

def aruco_drawer_id88(arg):

    output_file = './static/selected'
    file_name = os.path.splitext(output_file)[0]

    marker_size = arg

    w, h = marker_size, marker_size
    dwg = svgwrite.Drawing(filename=file_name + '.svg', size=(w*cm, h*cm), debug=True)
    dwg.add(dwg.rect(insert=(0,0), size=(w*cm, h*cm), fill='black', stroke='none'))

    #center of the marker
    cx = cy = marker_size/2

    #Squares for marker size reference
    corner_square = 0.137 * marker_size + 0.0196
    size_inner_square = 0.725 * marker_size - 0.392
    size_id_square_x = corner_square
    size_id_square_y = size_inner_square

    #We create a group for the whole marker
    marker = dwg.defs.add(dwg.g(id='marker'))

    #ID88
    #inner square
    marker.add(svgwrite.shapes.Rect((corner_square*cm,corner_square*cm),(size_inner_square*cm,size_inner_square*cm), fill='black'))

    #id square
    marker.add(svgwrite.shapes.Rect((corner_square*cm,corner_square*cm),(size_id_square_x*cm,3*size_id_square_x*cm), fill='white'))
    marker.add(svgwrite.shapes.Rect((corner_square*cm,5*corner_square*cm),(size_id_square_x*cm,size_id_square_x*cm), fill='white'))
    marker.add(svgwrite.shapes.Rect((2*corner_square*cm,4*corner_square*cm),(size_id_square_x*cm,size_id_square_x*cm), fill='white'))
    marker.add(svgwrite.shapes.Rect((3*corner_square*cm,2*corner_square*cm),(3*size_id_square_x*cm,2*size_id_square_x*cm), fill='white'))
    marker.add(svgwrite.shapes.Rect((5*corner_square*cm,4*corner_square*cm),(size_id_square_x*cm,size_id_square_x*cm), fill='white', stroke='white'))

    u = dwg.use(marker, insert=(0*cm,0*cm))
    dwg.add(u)
    dwg.save()

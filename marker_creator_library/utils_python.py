#!/usr/bin/env python

import timeit
import os
from os import rename, listdir

##################### TO BE MODIFIED #################
# Indicate where the library path
lib_path = "./static/"

# Indicate where to print drawing time results
print_comp_time = './results/time_results.txt'

# Indicate where to print file size results
print_file_size = './results/size_results.txt'

# List of all the expected file names to be used
array = ['aruco_marker_id0_14.svg', 'aruco_marker_id88_4.svg', 'aruco_marker_id88_14.svg', 'pitag_marker_r1_14.svg', 'pitag_marker_r05_14.svg']
######################################################

### Global variables ###
# Used to keep track of the current selected file for loading
file_selected = ''
# Used for calculating the processing time
time_start = 0
time_total = 0

def writeCompuTimeResultsInFile( arg ):
	with open( print_comp_time, 'a') as file_:
		file_.write('%s \n' % arg)
		return
def writeFileSizeResultsInFile( arg ):
	with open( print_file_size, 'a') as file_:
		file_.write('%s \n' % arg)
		return

# Store value of selected file
def findCurrentSelectedFile():
	fnames = listdir( lib_path ) # keep all file names in the dir: 'file_11', 'file_12' , 'selected' (for example)

	current = list(set(array) - set(fnames)) # returns differences between lists
	global file_selected # modify global variable
	file_selected = current[0] # keep value in string
	print "Current selected file is [%s]" % file_selected
	return

## Select a new file
# arg new_selection : string
def selectNewFile( new_selection ):
	global file_selected #indicate that it is the global value
	if new_selection == file_selected:
		print "File already selected"
		return False
	else:
		os.rename(lib_path + 'selected.svg', lib_path + file_selected) # old file gets back its name
		os.rename( lib_path + new_selection , lib_path + 'selected.svg' ) # new file renames to selected
		file_selected = new_selection # update global variable to keep track
		return True

## Calculate size of a file (inside lib directory)
# arg file : string
def calculateFileSize( file ):
	fileinfo = os.stat( lib_path + file );
	size_file = fileinfo.st_size;
	writeFileSizeResultsInFile( size_file );
	return size_file

# Clean previous stored results
def clearResultFiles():
	# Clear computation time results
	open(print_comp_time, 'w').close();
	# Clear size file results
	open(print_file_size, 'w').close();
	return

def startCount():
	global time_start
	time_start = timeit.default_timer();
	return

def stopCount():
	global time_total
	time_total = timeit.default_timer() - time_start;
	writeCompuTimeResultsInFile( time_total );
	print time_total
	return time_total

# Initialize
def init():
	findCurrentSelectedFile();
	clearResultFiles();
	return

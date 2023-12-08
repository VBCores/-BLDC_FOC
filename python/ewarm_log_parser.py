import numpy as np

def func( filename, start_word ):
    
    data = np.genfromtxt( filename+".log", dtype=str, delimiter='\t') 
    
    # search for the beginning of rows with calibration values
    while data[0][0] != start_word:
        data = np.delete(data, 0, axis=0) # delete the first row in array
        
    data = np.delete(data, 0, axis=0) # delete one more row containing "CalBuf_Fwd"
    data = np.delete(data, (0,2,3,4), axis=1) # delete columns 
    
    calibration_values = data[:1024] # copy only the calibration value rows
    
    for i in range( 1024 ): # remove ' delimiter
        calibration_values[i][0] = calibration_values[i][0].replace("'", "")
        
    calibration_values = calibration_values.astype(float) # convert a string array to float 
    calibration_values = np.array(calibration_values).ravel() # change array shape, delete "excess" brackets
    
    print(calibration_values.shape)
    print(calibration_values)
    
    np.savetxt( filename+".csv", calibration_values, delimiter="\n")

func("Watch1", "CalBuf_Fwd")
func("Watch2", "CalBuf_Bckwd")
import numpy as np
import matplotlib.pyplot as plt

class Plotter:
    def __init__(self, title, figsize=(10,10), offset=(0.05, 0.05)):
        self.title = title
        self.entries = {"Time":[[], "Time [s]"]}
        self.axes_to_plot = []
        
        self.figsize = figsize
        self.offset = offset
        
        self.num_rows = -1
        self.num_cols = -1
        
    def create_plot(self, key, ylabel):
        self.entries[key] = [[], ylabel]
    
    def add_data(self, key, data):
        self.entries[key][0].append(data)
    
    def add_time(self, time):
        self.entries["Time"][0].append(time)
        
    def create_axis(self, entries=[]):
        if len(entries) < 1:
            #No entries given
            #Add every array from self.entires, except "Time"
            for entry in self.entries:
                if entry != "Time":
                    entries.append(entry)
        
        elif isinstance(entries, str):
            #Single key given
            #Wrap the key in a list and add it
            
            entries = [entries]
            
        #If a list of strings is given, entries is allready correclty formated to add the specified keys
        self.axes_to_plot.append(entries)
        
    def get_entries_containing(self, search_word):
        entries_to_return = []
        
        for key in self.entries:
            if search_word in key:
                entries_to_return.append(key)
            
        return entries_to_return
    
    def plot(self, specified_entries=[]):
        #First, determine what to plot based on input argument
        entries_to_plot = []
        if len(specified_entries) < 1:
            #No argument was given
            if len(self.axes_to_plot) > 0:
                #Plot stored axes
                entries_to_plot = self.axes_to_plot
            
            else:
                #No stored axes, plot all entries in subplots
                for entry in self.entries:
                    if entry != "Time":
                        entry = [entry]
                        entries_to_plot.append(entry)

        else:
            #Argument given, plot specified entries
            entries_to_plot = specified_entries
        
        #Creating the figure and collecting stored size and offsets
        fig = plt.figure(figsize = self.figsize)
        x_offset = self.offset[0]
        y_offset = self.offset[1]
        
        #Making the shape
        num_entries = len(entries_to_plot)
        
        if self.num_cols != -1 and self.num_rows != -1:
            num_cols = self.num_cols
            num_rows = self.num_rows
            self.num_cols = -1
            self.num_rows = -1

        elif self.num_cols != -1:
            num_cols = self.num_cols
            self.num_cols = -1

            num_rows = int(np.ceil(num_entries / num_cols))

        elif self.num_rows != -1:
            num_rows = self.num_rows
            self.num_rows = -1

            num_cols = int(np.ceil(num_entries / num_rows))

        else:
            if num_entries <= 3:
                num_rows = 1
                num_cols = num_entries
            else:
                num_rows = int(np.ceil(num_entries / 3))
                num_cols = 3
        
        #print(num_rows, num_cols)

        #Plotting all entries in entries_to_plot
        size_x = 1 / num_cols
        size_y = 1 / num_rows
        
        y = 0
        x = 0
        
        for keys in entries_to_plot: #Keys is now a list of entries, corresponing to an axis
            legend_space = len(keys) * 0.025

            if y_offset < legend_space:
                y_offset = legend_space

            pos_x = x * size_x + x_offset
            pos_y = y * size_y + y_offset
        
            axes_size_x = size_x - 2 * x_offset
            axes_size_y = size_y - 2 * y_offset
        
            axes = fig.add_axes([pos_x, pos_y, axes_size_x, axes_size_y])
            
            for key in keys:
                x_data = self.entries["Time"][0]
                y_data = self.entries[key][0]
                
                if len(x_data) > len(y_data):
                    #print(f"{len(x_data) - len(y_data)} values were dropped from x_data")
                    x_data = x_data[:len(y_data)]
                elif len(y_data) > len(x_data):
                    #print(f"{len(y_data) - len(x_data)} values were dropped from y_data")
                    y_data = y_data[:len(x_data)]
                
                axes.plot(x_data, y_data, label=key)
                    
            axes.set_xlabel(self.entries["Time"][1])
            axes.set_ylabel(self.entries[key][1])
            axes.legend(loc=(0,1))
            axes.grid()
            
            if x < num_cols - 1:
                x += 1
            else:
                x = 0
                y += 1
        
        
        self.axes_to_plot = []
        #plt.grid()
        plt.show()
        
    def set_num_rows(self, n):
        self.num_rows = n
    
    def set_num_cols(self, n):
        self.num_cols = n
    
    def set_shape(self, shape):
        self.num_rows = shape[0]
        self.num_cols = shape[1]
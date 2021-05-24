import matplotlib.pyplot as plt

def overlay_plot(p_array, v_array, p_ref_array, v_ref_array, p_err_array, v_err_array, t_array, current_array, Kp, Kd):
    """
    Plotting position and velocity data in two separate subplots.
    Does have dark mode ON
    """
    fig, axs = plt.subplots(3, 3)
    fig.set_figheight(75)
    fig.set_figwidth(75)
    plt.suptitle(f"Bang Bang Acceleration profile (Kp = {Kp} | Kd = {Kd})", fontsize="24") # You can choose a title for the plots here
    #mpl.style.use("dark_background") # Comment out this if you don't like the dark background

    # Position subplot
    axs[0,0].plot(t_array, p_ref_array) # Plots the desired position that is given to the motor
    axs[0,0].set_title('PositionReference')
    axs[0,0].set(xlabel='Time [ms]', ylabel='Position [rad]')
    axs[0,1].plot(t_array, p_array) # Plots the measured position
    axs[0,1].set_title('GetPosition')
    axs[0,1].set(xlabel='Time [ms]', ylabel='Position [rad]')
    axs[0,2].plot(t_array, p_err_array) # Plots position error
    axs[0,2].set_title('Position error')
    axs[0,2].set(xlabel='Time [ms]', ylabel='Position [rad]')

    # Velocity subplot
    axs[1,0].plot(t_array, v_ref_array) # 
    axs[1,0].set_title('VelocityReference')
    axs[1,0].set(xlabel='Time [ms]', ylabel='Velocity [rad/s]')
    axs[1,1].plot(t_array, v_array)
    axs[1,1].set_title('GetVelocity')
    axs[1,1].set(xlabel='Time [ms]', ylabel='Velocity [rad/s]')
    axs[1,2].plot(t_array, v_err_array)  
    axs[1,2].set_title('Velocity error')
    axs[1,2].set(xlabel='Time [ms]', ylabel='Velocity [rad/subplot]')

    #Current subplot
    axs[2,0].plot(t_array, current_array) # 
    axs[2,0].set_title('Current Reference')
    axs[2,0].set(xlabel='Time [ms]', ylabel='Current [Ampere]')

    plt.show()
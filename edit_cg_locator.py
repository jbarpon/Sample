
# Locates the centre of gravity of the aircraft and the CGs of its various components
# Estimates weights of the different components

# random comment 2

# Author: Arpon, Joshua

# Sources: 


# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

import numpy as np
from SUAVE.Methods.Geometry.Three_Dimensional.compute_span_location_from_chord_length import \
    compute_span_location_from_chord_length
from SUAVE.Methods.Geometry.Three_Dimensional.compute_chord_length_from_span_location import \
    compute_chord_length_from_span_location
from SUAVE.Methods.Flight_Dynamics.Static_Stability.Approximations.Supporting_Functions.convert_sweep import \
    convert_sweep

import SUAVE

import tut_concorde

vehicle= tut_concorde.vehicle_setup()


# ----------------------------------------------------------------------
#  Computer Aircraft Center of Gravity
# ----------------------------------------------------------------------

def CG_Locator(vehicle):
    """ computes the CG of all of the vehicle components based on correlations
    from AA241. aircraft CG is calculated with weights breakdown of components
    Assumptions:
    vehicle is symmetric (CG is in X). total weight is empty weight. 
    Source:
    AA 241 Notes
    Inputs:
    vehicle
    Outputs:
    Total_Aircraft_CG
    Properties Used:
    N/A
    """
    
    C = SUAVE.Components

    # Initilizing Arrays of CG Locations
    Main_Wing_Locations = np.array([])
    Horizontal_Tail_Locations = np.array([])
    Vertical_Tail_Locations = np.array([])
    Fuselage_Locations = np.array([])
    Propulsor_Locations = np.array([])

    for part in vehicle.keys():

        # Wing Handler
        if part == 'wings':

            wings = vehicle.wings

            for wing_type in wings:
                if wing_type.sweeps.leading_edge == None:
                    wing_type.sweeps.leading_edge = convert_sweep(wing_type, old_ref_chord_fraction=0.25, new_ref_chord_fraction=0.0)
                    

                if isinstance(wing_type,C.Wings.Main_Wing):
                    wing_type.mass_properties.center_of_gravity[0][0] = .05 * wing_type.chords.mean_aerodynamic + wing_type.aerodynamic_center[0]
                    Main_Wing_Locations = np.append(Main_Wing_Locations,wing_type.mass_properties.center_of_gravity[0][0])

                elif isinstance(wing_type,C.Wings.Horizontal_Tail):

                    chord_length_h_tail_35_percent_semi_span = compute_chord_length_from_span_location(wing_type,
                                                                                                        .35 * wing_type.spans.projected * .5)
                    h_tail_35_percent_semi_span_offset = np.tan(wing_type.sweeps.quarter_chord) * .35 * .5 * wing_type.spans.projected
                    wing_type.mass_properties.center_of_gravity[0][0] = .3 * chord_length_h_tail_35_percent_semi_span + \
                                                                    h_tail_35_percent_semi_span_offset   

                    Horizontal_Tail_Locations = np.append(Horizontal_Tail_Locations,wing_type.mass_properties.center_of_gravity[0][0])

                elif isinstance(wing_type,C.Wings.Vertical_Tail):

                    chord_length_v_tail_35_percent_semi_span = compute_chord_length_from_span_location(wing_type,
                                                                                                    .35 * wing_type.spans.projected)
                    v_tail_35_percent_semi_span_offset = np.tan(wing_type.sweeps.quarter_chord) * .35 * .5 * wing_type.spans.projected
                    wing_type.mass_properties.center_of_gravity[0][0] = .3 * chord_length_v_tail_35_percent_semi_span + \
                                                                v_tail_35_percent_semi_span_offset

                    Vertical_Tail_Locations = np.append(Vertical_Tail_Locations,wing_type.mass_properties.center_of_gravity[0][0])

        # Propulsor Handler
        elif part == 'propulsors':

            for prop in vehicle.propulsors:
                prop.mass_properties.center_of_gravity[0][0] = prop.engine_length*0.5
                Propulsor_Locations = np.append(Propulsor_Locations,prop.mass_properties.center_of_gravity[0][0])
                                 
        # Fuselage handler
        elif part =='fuselages':

            for fuselage in vehicle.fuselages:
                fuselage.mass_properties.center_of_gravity[0][0] = 0.45*fuselage.lengths.total
                Fuselage_Locations = np.append(Fuselage_Locations,fuselage.mass_properties.center_of_gravity[0][0])

        # Miscellaneous Component Handler
        else: 

            if (part != 'costs') and (part != 'envelope') and (part != 'mass_properties') and (part != 'reference_area') and (part != 'performance') \
                and (part != 'tag') and (part != 'maximum_cross_sectional_area'):

                print(part)

                # Not sure how to handle the miscellanous items. Right now, the weight of the misc items are just put on the nose.
                           
    # end for-loop
    
    # Calculating the CG
    Total_Aircraft_CG = 0
    
    # Main Wing is 30%, Fuel tank is 5%
    if  len(Main_Wing_Locations)  > 0:
        Main_Wing_CG_Array_Length = len(Main_Wing_Locations) 
        Main_Wing_CG = (0.3/Main_Wing_CG_Array_Length)*np.sum(Main_Wing_Locations)

        # Fuel tank located in wings
        Fuel_Tank_CG_Locations = Main_Wing_Locations
        Fuel_Tank_CG_Array_Length = Main_Wing_CG_Array_Length
        Fuel_Tank_CG = (0.05/Fuel_Tank_CG_Array_Length)*np.sum(Fuel_Tank_CG_Locations) 

        Total_Aircraft_CG +=  Main_Wing_CG + Fuel_Tank_CG

    # Tails are 10%
    if len(Horizontal_Tail_Locations) > 0:
        Horizontal_Tail_Array_Lengths = len(Horizontal_Tail_Locations) 
        Horizontal_Tail_CG = (0.025/Horizontal_Tail_Array_Lengths)*np.sum(Horizontal_Tail_Locations) 
        Total_Aircraft_CG +=  Horizontal_Tail_CG

    if len(Vertical_Tail_Locations) > 0:
        Vertical_Tail_Array_Length = len(Vertical_Tail_Locations) 
        Vertical_Tail_CG = (0.025/Vertical_Tail_Array_Length)*np.sum(Vertical_Tail_Locations)
        Total_Aircraft_CG += Vertical_Tail_CG

    # Fuselage is 33%
    if len(Fuselage_Locations) > 0:
        Fuselage_Array_Length = len(Fuselage_Locations) 
        Fuselage_CG = (0.33/Fuselage_Array_Length)*np.sum(Fuselage_Locations)
        Total_Aircraft_CG += Fuselage_CG

    # Propulsor is 7%
    if len(Propulsor_Locations)  > 0:
        Propulsor_Array_Length = len(Propulsor_Locations) 
        Propulsor_CG = (0.07/Propulsor_Array_Length)*np.sum(Fuselage_Locations)
        Total_Aircraft_CG  += Propulsor_CG

    # I need a condition for Dry Engine

    # Misc items are 10% of total weight and are located at point 0
    
    return Total_Aircraft_CG

x = CG_Locator(vehicle)
print(x)




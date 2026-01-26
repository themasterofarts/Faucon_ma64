import utm

class LocalFrame():
    
    """ Cette classe consiste à récupére les coordonné gps pour ensuite les convertir en 
    cordonnée local(x, y).
    
    La première mesure GNSS fixe l’origine en UTM.
    
    puis les mesures suivantes sont converties en coordonnées locales (x,y) par différence avec cette origine.
    """
    
    ### constructeur 
    
    def __init__(self) :
        self.origin_set = False
        self.x0 = 0.0
        self.y0 = 0.0
        
        
    
    #### Methode qui recupere les coordonée gps ####
    
    def gnss_to_local(self, lat, lon):
        
        
        #### convertion des cordonnées gps en utm ###
        
        easting, northing, zone, center = utm.from_latlon(lat,lon)
        
        #### fixation de l'origine local
        if not self.origin_set:
            self.x0 = easting
            self.y0 = northing
            self.origin_set = True
        
    
    #### methode pour la conversion en utm        
        ### dans cette parti on effectue la conversion
        
        x_local = easting - self.x0 
        y_local = northing - self.y0 
        
        return x_local, y_local
    
    
        
package matlab.generator.area;

import java.util.Collection;
import java.util.Set;
import rescuecore2.misc.Pair;
import rescuecore2.standard.entities.Area;
import rescuecore2.worldmodel.EntityID;

/* The Sector object, assigned to each Police Force */
public class Sector implements Comparable<Sector> {
  
  int           index;
  
  int           minX;
  
  int           minY;
  
  int           maxX;
  
  int           maxY;
  
  Set<EntityID> locations;
  
  
  public Sector(int minx, int miny, int maxx, int maxy, int i) {
    minX = minx;
    minY = miny;
    maxX = maxx;
    maxY = maxy;
    index = i;
    locations = null;
  }
  
  
  public Sector(Sector other) {
    minX = other.minX;
    minY = other.minY;
    maxX = other.maxX;
    maxY = other.maxY;
    index = other.index;
    locations = other.locations;
  }
  
  
  /*
   * Get the bounds of a sector.
   * @return A pair containing the two pairs: the first represents the top-left
   * corner and the second represents the bottom-right corner of the sector.
   */
  public Pair<Pair<Integer, Integer>, Pair<Integer, Integer>> getBounds() {
    Pair<Integer, Integer> minimum = new Pair<Integer, Integer>(
        new Integer(minX), new Integer(minY));
    Pair<Integer, Integer> maximum = new Pair<Integer, Integer>(
        new Integer(maxX), new Integer(maxY));
    
    Pair<Pair<Integer, Integer>, Pair<Integer, Integer>> bounds = new Pair<Pair<Integer, Integer>, Pair<Integer, Integer>>(
        minimum, maximum);
    
    return bounds;
  }
  
  
  public void setLocations(Set<EntityID> other) {
    locations = other;
  }
  
  
  public Set<EntityID> getLocations() {
    return locations;
  }
  
  
  public int getIndex() {
    return index;
  }
  
  
  public void setIndex(int i) {
    index = i;
  }
  
  
  public void addVertex(EntityID v) {
    locations.add(v);
  }
  
  
  public void addVertices(Collection<EntityID> c) {
    locations.addAll(c);
  }
  
  
  @Override
  public String toString() {
    return (new Integer(index)).toString();
  }
  
  
  public boolean geographicallyContains(Area entity) {
    if(entity.getX() >= minX && entity.getY() >= minY && entity.getX() <= maxX
        && entity.getY() <= maxY) {
      return true;
    }
    
    return false;
  }
  
  
  @Override
  public int compareTo(Sector o) {
    int i = o.getIndex();
    
    if(index < i) {
      return -1;
    }
    if(index > i) {
      return 1;
    }
    return 0;
  }
}

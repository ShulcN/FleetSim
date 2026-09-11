"""Usage: python examples/swagger_output/convert_gml_to_geojson.py graph.gml graph.geojson [options].
Coordinates are local metres. Input is preserved; --scale/--rotation/--offset register it to the DXF.
"""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from backend.research.graph_io import main
if __name__=='__main__':main()

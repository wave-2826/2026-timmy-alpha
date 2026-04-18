import vlogger
import urllib

split = urllib.parse.urlsplit("wpilog://./log.wpilog")
print(split.path.lstrip('/'))

# "" regex matches with anything, i.e. any field
source = vlogger.get_source("wpilog://./log.wpilog", "")
with source as source:
    for field in source:
        print(field)

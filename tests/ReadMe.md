# SwiftTest.py on windows : 

replace ligne from SwiftRoute.py which contain :
```Python
self.path = urllib.parse.unquote(self.path[9:])
```

by 

```Python
self.path = urllib.parse.unquote(self.path[10:])
```
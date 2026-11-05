import assert from 'node:assert/strict';
import {existsSync, mkdtempSync, mkdirSync, readFileSync, rmSync, writeFileSync} from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import {createWebCopy} from '../scripts/create-web-copy.mjs';

const fixture=mkdtempSync(path.join(os.tmpdir(),'et2-web-copy-'));
try{
  mkdirSync(path.join(fixture,'src'));
  mkdirSync(path.join(fixture,'scripts'));
  mkdirSync(path.join(fixture,'tests'));
  mkdirSync(path.join(fixture,'web2'));
  mkdirSync(path.join(fixture,'.git'));
  writeFileSync(path.join(fixture,'index.html'),'<link rel="stylesheet" href="site.css"><script src="model.js"></script><a href="joint.html">关节</a>');
  writeFileSync(path.join(fixture,'joint.html'),'<a href="index.html">首页</a>');
  writeFileSync(path.join(fixture,'site.css'),'body{}');
  writeFileSync(path.join(fixture,'model.js'),'window.model=true');
  writeFileSync(path.join(fixture,'src','model.js'),'export const model=true');
  writeFileSync(path.join(fixture,'scripts','build.mjs'),'export {}');
  writeFileSync(path.join(fixture,'tests','model.test.mjs'),'// test');
  writeFileSync(path.join(fixture,'web2','stale.txt'),'must not be recursively copied');
  writeFileSync(path.join(fixture,'.git','config'),'private');

  const destination=path.join(fixture,'web2');
  createWebCopy({root:fixture,destination,sourceCommit:'abc1234'});

  for(const relative of ['index.html','joint.html','site.css','model.js','src/model.js','scripts/build.mjs','tests/model.test.mjs','WEBPAGE_VERSION.md']){
    assert.equal(existsSync(path.join(destination,...relative.split('/'))),true,`${relative} should be copied`);
  }
  assert.equal(existsSync(path.join(destination,'.git','config')),false,'.git must not be copied');
  assert.equal(existsSync(path.join(destination,'web2','stale.txt')),false,'destination must not copy itself');
  assert.match(readFileSync(path.join(destination,'WEBPAGE_VERSION.md'),'utf8'),/abc1234/);
  const html=readFileSync(path.join(destination,'index.html'),'utf8');
  assert.doesNotMatch(html,/(?:href|src)=["'](?:\.\.\/|\/)/,'copy must not reference outside its directory');
}finally{
  rmSync(fixture,{recursive:true,force:true});
}

console.log('web copy regression test passed');
